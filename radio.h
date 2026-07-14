// Pod sub-GHz radio — beacons (presence) + battle-control messages, all on
// 433.92 MHz via the half-duplex TX/RX worker.
#pragma once

#include "profile.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    PodMsgBeacon = 0x01, // presence broadcast
    PodMsgChallenge = 0x02, // src wants to battle dst
    PodMsgAccept = 0x03, // dst accepts the challenge
    PodMsgDecline = 0x04, // dst declines
    PodMsgTaps = 0x05, // final tap count exchange
} PodMsgType;

// A decoded radio message. Which fields are meaningful depends on `type`.
typedef struct {
    uint8_t type;
    uint32_t src_id;
    uint32_t dst_id; // control messages
    uint16_t battle_id; // control messages
    uint16_t taps; // TAPS
    uint8_t level; // beacon/challenge/accept
    uint16_t nonce; // beacon
    uint8_t icon; // beacon
    char name[POD_NAME_MAX + 1]; // beacon/challenge/accept
} PodMsg;

typedef struct PodRadio PodRadio;

// Called on the radio worker thread with each de-duplicated message. Keep light.
typedef void (*PodMsgCallback)(void* context, const PodMsg* msg);

PodRadio* pod_radio_alloc(void);
void pod_radio_free(PodRadio* radio);
void pod_radio_set_rx_callback(PodRadio* radio, PodMsgCallback cb, void* context);

bool pod_radio_start(PodRadio* radio);
void pod_radio_stop(PodRadio* radio);
bool pod_radio_is_running(PodRadio* radio);

// Sends (each transmitted a few times for redundancy). Return true if queued.
bool pod_radio_beacon(PodRadio* radio, const PodProfile* profile, uint8_t dolphin_level);
bool pod_radio_send_challenge(
    PodRadio* radio,
    uint32_t src,
    const char* name,
    uint8_t level,
    uint32_t dst,
    uint16_t battle_id);
bool pod_radio_send_accept(
    PodRadio* radio,
    uint32_t src,
    const char* name,
    uint8_t level,
    uint32_t dst,
    uint16_t battle_id);
bool pod_radio_send_decline(PodRadio* radio, uint32_t src, uint32_t dst, uint16_t battle_id);
bool pod_radio_send_taps(
    PodRadio* radio,
    uint32_t src,
    uint32_t dst,
    uint16_t battle_id,
    uint16_t taps);
