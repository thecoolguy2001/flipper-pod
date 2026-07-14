#include "radio.h"

#include <furi.h>
#include <furi_hal_random.h>
#include <lib/subghz/subghz_tx_rx_worker.h>
#include <lib/subghz/devices/devices.h>
#include <lib/subghz/devices/cc1101_int/cc1101_int_interconnect.h>
#include <string.h>

#define POD_FREQUENCY 433920000UL // 433.92 MHz ISM, region-legal
#define POD_FRAME_MAX 48
#define POD_PARSE_BUF 160
#define POD_DEDUP_N 24
#define POD_SEND_REPEAT 3

typedef struct {
    uint32_t src;
    uint8_t type;
    uint16_t key2; // nonce for beacon, battle_id for control
} PodSeen;

struct PodRadio {
    SubGhzTxRxWorker* worker;
    const SubGhzDevice* device;
    bool running;
    PodMsgCallback rx_cb;
    void* rx_ctx;
    uint8_t parse_buf[POD_PARSE_BUF];
    size_t parse_len;
    PodSeen recent[POD_DEDUP_N];
    int recent_pos;
};

// CRC-8, poly 0x07 — matches protocol.md.
static uint8_t pod_crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for(size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for(uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static size_t put_u32(uint8_t* p, uint32_t v) {
    p[0] = v & 0xFF;
    p[1] = (v >> 8) & 0xFF;
    p[2] = (v >> 16) & 0xFF;
    p[3] = (v >> 24) & 0xFF;
    return 4;
}

static uint32_t get_u32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static size_t name_len(const char* name) {
    size_t n = 0;
    while(n < POD_NAME_MAX && name[n]) n++;
    return n;
}

// --- lifecycle ---

PodRadio* pod_radio_alloc(void) {
    PodRadio* radio = malloc(sizeof(PodRadio));
    radio->worker = subghz_tx_rx_worker_alloc();
    radio->device = NULL;
    radio->running = false;
    radio->rx_cb = NULL;
    radio->rx_ctx = NULL;
    radio->parse_len = 0;
    memset(radio->recent, 0, sizeof(radio->recent));
    radio->recent_pos = 0;
    return radio;
}

void pod_radio_free(PodRadio* radio) {
    if(radio->running) pod_radio_stop(radio);
    subghz_tx_rx_worker_free(radio->worker);
    free(radio);
}

void pod_radio_set_rx_callback(PodRadio* radio, PodMsgCallback cb, void* context) {
    radio->rx_cb = cb;
    radio->rx_ctx = context;
}

// --- receive ---

static bool pod_radio_seen(PodRadio* radio, uint32_t src, uint8_t type, uint16_t key2) {
    for(int k = 0; k < POD_DEDUP_N; k++) {
        if(radio->recent[k].src == src && radio->recent[k].type == type &&
           radio->recent[k].key2 == key2)
            return true;
    }
    radio->recent[radio->recent_pos].src = src;
    radio->recent[radio->recent_pos].type = type;
    radio->recent[radio->recent_pos].key2 = key2;
    radio->recent_pos = (radio->recent_pos + 1) % POD_DEDUP_N;
    return false;
}

// Total on-wire length of the message starting at p (len bytes available), or 0
// if incomplete/invalid. On success also fills *namelen_out for name-bearing types.
static size_t pod_msg_len(const uint8_t* p, size_t len) {
    if(len < 3) return 0;
    switch(p[2]) {
    case PodMsgBeacon:
        if(len < 12) return 0;
        if(p[11] > POD_NAME_MAX) return SIZE_MAX; // invalid -> caller resyncs
        return 12 + p[11] + 1;
    case PodMsgChallenge:
    case PodMsgAccept:
        if(len < 15) return 0;
        if(p[14] > POD_NAME_MAX) return SIZE_MAX;
        return 15 + p[14] + 1;
    case PodMsgDecline:
        return 14;
    case PodMsgTaps:
        return 16;
    default:
        return SIZE_MAX; // unknown type -> resync
    }
}

static void pod_msg_decode(const uint8_t* p, PodMsg* m) {
    memset(m, 0, sizeof(*m));
    m->type = p[2];
    m->src_id = get_u32(&p[3]);
    switch(p[2]) {
    case PodMsgBeacon: {
        m->icon = p[7];
        m->level = p[8];
        m->nonce = (uint16_t)(p[9] | (p[10] << 8));
        uint8_t nl = p[11];
        memcpy(m->name, &p[12], nl);
        m->name[nl] = '\0';
        break;
    }
    case PodMsgChallenge:
    case PodMsgAccept: {
        m->dst_id = get_u32(&p[7]);
        m->level = p[11];
        m->battle_id = (uint16_t)(p[12] | (p[13] << 8));
        uint8_t nl = p[14];
        memcpy(m->name, &p[15], nl);
        m->name[nl] = '\0';
        break;
    }
    case PodMsgDecline:
        m->dst_id = get_u32(&p[7]);
        m->battle_id = (uint16_t)(p[11] | (p[12] << 8));
        break;
    case PodMsgTaps:
        m->dst_id = get_u32(&p[7]);
        m->battle_id = (uint16_t)(p[11] | (p[12] << 8));
        m->taps = (uint16_t)(p[13] | (p[14] << 8));
        break;
    default:
        break;
    }
}

static void pod_radio_parse(PodRadio* radio) {
    size_t i = 0;
    while(radio->parse_len - i >= 3) {
        uint8_t* p = &radio->parse_buf[i];
        if(!(p[0] == 'P' && p[1] == 'D')) {
            i++;
            continue;
        }
        size_t total = pod_msg_len(p, radio->parse_len - i);
        if(total == 0) break; // need more bytes
        if(total == SIZE_MAX) {
            i++;
            continue;
        } // invalid -> resync
        if(pod_crc8(p, total - 1) != p[total - 1]) {
            i++;
            continue;
        }
        PodMsg m;
        pod_msg_decode(p, &m);
        i += total;
        uint16_t key2 = (m.type == PodMsgBeacon) ? m.nonce : m.battle_id;
        if(!pod_radio_seen(radio, m.src_id, m.type, key2) && radio->rx_cb) {
            radio->rx_cb(radio->rx_ctx, &m);
        }
    }
    if(i > 0) {
        memmove(radio->parse_buf, radio->parse_buf + i, radio->parse_len - i);
        radio->parse_len -= i;
    }
}

static void pod_radio_have_read(void* context) {
    PodRadio* radio = context;
    size_t avail = subghz_tx_rx_worker_available(radio->worker);
    while(avail > 0) {
        uint8_t tmp[64];
        size_t n = subghz_tx_rx_worker_read(radio->worker, tmp, MIN(avail, sizeof(tmp)));
        if(n == 0) break;
        for(size_t k = 0; k < n; k++) {
            if(radio->parse_len < POD_PARSE_BUF) {
                radio->parse_buf[radio->parse_len++] = tmp[k];
            } else {
                memmove(radio->parse_buf, radio->parse_buf + 1, POD_PARSE_BUF - 1);
                radio->parse_buf[POD_PARSE_BUF - 1] = tmp[k];
            }
        }
        avail = subghz_tx_rx_worker_available(radio->worker);
    }
    pod_radio_parse(radio);
}

bool pod_radio_start(PodRadio* radio) {
    if(radio->running) return true;
    subghz_devices_init();
    radio->device = subghz_devices_get_by_name(SUBGHZ_DEVICE_CC1101_INT_NAME);
    if(!radio->device) {
        subghz_devices_deinit();
        return false;
    }
    if(!subghz_tx_rx_worker_start(radio->worker, radio->device, POD_FREQUENCY)) {
        subghz_devices_deinit();
        radio->device = NULL;
        return false;
    }
    radio->parse_len = 0;
    subghz_tx_rx_worker_set_callback_have_read(radio->worker, pod_radio_have_read, radio);
    radio->running = true;
    return true;
}

void pod_radio_stop(PodRadio* radio) {
    if(!radio->running) return;
    if(subghz_tx_rx_worker_is_running(radio->worker)) {
        subghz_tx_rx_worker_stop(radio->worker);
    }
    subghz_devices_deinit();
    radio->device = NULL;
    radio->running = false;
}

bool pod_radio_is_running(PodRadio* radio) {
    return radio->running;
}

// --- send ---

static bool pod_radio_emit(PodRadio* radio, uint8_t* buf, size_t len, int repeat) {
    if(!radio->running) return false;
    bool ok = false;
    for(int r = 0; r < repeat; r++) {
        ok = subghz_tx_rx_worker_write(radio->worker, buf, len) || ok;
    }
    return ok;
}

bool pod_radio_beacon(PodRadio* radio, const PodProfile* profile, uint8_t dolphin_level) {
    uint8_t buf[POD_FRAME_MAX];
    uint16_t nonce = (uint16_t)(furi_hal_random_get() & 0xFFFF);
    size_t nl = name_len(profile->name);
    size_t i = 0;
    buf[i++] = 'P';
    buf[i++] = 'D';
    buf[i++] = PodMsgBeacon;
    i += put_u32(&buf[i], profile->profile_id);
    buf[i++] = profile->icon;
    buf[i++] = dolphin_level;
    buf[i++] = nonce & 0xFF;
    buf[i++] = (nonce >> 8) & 0xFF;
    buf[i++] = (uint8_t)nl;
    memcpy(&buf[i], profile->name, nl);
    i += nl;
    buf[i] = pod_crc8(buf, i);
    i++;
    return pod_radio_emit(radio, buf, i, 2);
}

static bool pod_radio_send_named(
    PodRadio* radio,
    uint8_t type,
    uint32_t src,
    const char* name,
    uint8_t level,
    uint32_t dst,
    uint16_t battle_id) {
    uint8_t buf[POD_FRAME_MAX];
    size_t nl = name_len(name);
    size_t i = 0;
    buf[i++] = 'P';
    buf[i++] = 'D';
    buf[i++] = type;
    i += put_u32(&buf[i], src);
    i += put_u32(&buf[i], dst);
    buf[i++] = level;
    buf[i++] = battle_id & 0xFF;
    buf[i++] = (battle_id >> 8) & 0xFF;
    buf[i++] = (uint8_t)nl;
    memcpy(&buf[i], name, nl);
    i += nl;
    buf[i] = pod_crc8(buf, i);
    i++;
    return pod_radio_emit(radio, buf, i, POD_SEND_REPEAT);
}

bool pod_radio_send_challenge(
    PodRadio* radio,
    uint32_t src,
    const char* name,
    uint8_t level,
    uint32_t dst,
    uint16_t battle_id) {
    return pod_radio_send_named(radio, PodMsgChallenge, src, name, level, dst, battle_id);
}

bool pod_radio_send_accept(
    PodRadio* radio,
    uint32_t src,
    const char* name,
    uint8_t level,
    uint32_t dst,
    uint16_t battle_id) {
    return pod_radio_send_named(radio, PodMsgAccept, src, name, level, dst, battle_id);
}

bool pod_radio_send_decline(PodRadio* radio, uint32_t src, uint32_t dst, uint16_t battle_id) {
    uint8_t buf[POD_FRAME_MAX];
    size_t i = 0;
    buf[i++] = 'P';
    buf[i++] = 'D';
    buf[i++] = PodMsgDecline;
    i += put_u32(&buf[i], src);
    i += put_u32(&buf[i], dst);
    buf[i++] = battle_id & 0xFF;
    buf[i++] = (battle_id >> 8) & 0xFF;
    buf[i] = pod_crc8(buf, i);
    i++;
    return pod_radio_emit(radio, buf, i, POD_SEND_REPEAT);
}

bool pod_radio_send_taps(
    PodRadio* radio,
    uint32_t src,
    uint32_t dst,
    uint16_t battle_id,
    uint16_t taps) {
    uint8_t buf[POD_FRAME_MAX];
    size_t i = 0;
    buf[i++] = 'P';
    buf[i++] = 'D';
    buf[i++] = PodMsgTaps;
    i += put_u32(&buf[i], src);
    i += put_u32(&buf[i], dst);
    buf[i++] = battle_id & 0xFF;
    buf[i++] = (battle_id >> 8) & 0xFF;
    buf[i++] = taps & 0xFF;
    buf[i++] = (taps >> 8) & 0xFF;
    buf[i] = pod_crc8(buf, i);
    i++;
    return pod_radio_emit(radio, buf, i, POD_SEND_REPEAT);
}
