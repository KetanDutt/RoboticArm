/*
 * RfLink.h -- a thin, driver-agnostic wrapper around the ASK radio.
 *
 * The project has always used VirtualWire.  VirtualWire is end-of-life
 * upstream and has been replaced by RadioHead, whose RH_ASK driver uses the
 * same framing; both are supported here behind one switch so the firmware is
 * not welded to an unmaintained library.  Set GLOVE_RF_DRIVER in
 * GloveConfig.h -- and set it to the SAME value on both boards, because a
 * VirtualWire transmitter and a RadioHead receiver will not interoperate
 * reliably in practice.
 *
 * The wrapper also fixes two things that the original sketches got wrong:
 *
 *   - sending is non-blocking.  The old code called vw_wait_tx() plus
 *     delay(10) after every frame, so the glove stopped sampling its sensors
 *     for the whole ~90 ms air time.  Here the caller checks txBusy() and
 *     simply keeps the newest sample if the radio is still working, which is
 *     exactly what a "latest state wins" control link wants.
 *   - receive() documents that *len is an in/out parameter.  The old
 *     receiver set it once in setup() and never restored it, so after the
 *     first packet it only ever copied that many bytes again.
 *
 * This header needs the Arduino API, so unlike the rest of `common/` it is
 * not compiled by the native tests.
 *
 * Shared file: edit `common/RfLink.h` and run `python3 tools/sync_common.py`;
 * the copies inside `hand_transmit/` and `hand_receive/` are generated and
 * are verified to match by CI.
 *
 * This file is part of the RoboticArm project.  See docs/PROTOCOL.md.
 */
#ifndef RFLINK_H
#define RFLINK_H

#include <Arduino.h>

#include "GloveConfig.h"
#include "GloveProtocol.h"

#if GLOVE_RF_DRIVER == GLOVE_RF_DRIVER_RH
#include <RH_ASK.h>
#else
#include <VirtualWire.h>
#endif

namespace rflink {

#if GLOVE_RF_DRIVER == GLOVE_RF_DRIVER_RH
/* One radio object for the whole sketch.  A function-local static inside an
 * inline function keeps this header safe to include from more than one
 * translation unit. */
inline RH_ASK &radio() {
    static RH_ASK driver(GLOVE_RF_SPEED_BPS, GLOVE_RF_RX_PIN, GLOVE_RF_TX_PIN,
                         GLOVE_RF_PTT_PIN, GLOVE_RF_PTT_INVERTED != 0);
    return driver;
}
#endif

/* Air time of one frame in microseconds.
 *
 * VirtualWire/RH_ASK put 36 bits of training preamble and a 12-bit start
 * symbol on the air, then encode every byte -- the length byte, the payload
 * and the two FCS bytes -- as two 6-bit symbols, i.e. 12 bits per byte:
 *
 *     bits = 48 + 12 * (1 + payloadLen + 2)
 *
 * A 9-byte frame is therefore 192 bits: 96 ms at 2000 bps, 48 ms at 4000 bps.
 * This number is the hard floor on the control loop latency, which is why
 * docs/PERFORMANCE.md treats payload size as a design parameter. */
inline uint32_t frameAirTimeUs(uint8_t payloadLen) {
    const uint32_t bits = 48u + 12u * (3u + static_cast<uint32_t>(payloadLen));
    return (bits * 1000000u) / static_cast<uint32_t>(GLOVE_RF_SPEED_BPS);
}

/* Software interlock used when the driver does not expose its TX state. */
inline uint32_t &txDeadlineUs() {
    static uint32_t deadline = 0;
    return deadline;
}

inline const char *driverName() {
#if GLOVE_RF_DRIVER == GLOVE_RF_DRIVER_RH
    return "RadioHead/RH_ASK";
#else
    return "VirtualWire";
#endif
}

/* Configure pins and data rate.  Call once from setup(). */
inline void begin() {
#if GLOVE_RF_DRIVER == GLOVE_RF_DRIVER_RH
    radio().init();
#else
    vw_set_tx_pin(GLOVE_RF_TX_PIN);
    vw_set_rx_pin(GLOVE_RF_RX_PIN);
    vw_set_ptt_pin(GLOVE_RF_PTT_PIN);
#if GLOVE_RF_PTT_INVERTED
    vw_set_ptt_inverted(true);
#endif
    vw_setup(static_cast<uint16_t>(GLOVE_RF_SPEED_BPS));
#endif
}

/* Start the receive interrupt.  Only the arm calls this; leaving it off on
 * the glove keeps the Timer1 ISR out of the way of the sensor sampling. */
inline void startRx() {
#if GLOVE_RF_DRIVER != GLOVE_RF_DRIVER_RH
    vw_rx_start();
#endif
}

/* True while a frame is still going out. */
inline bool txBusy() {
#if GLOVE_RF_DRIVER == GLOVE_RF_DRIVER_RH
    return static_cast<int32_t>(txDeadlineUs() - micros()) > 0;
#else
    return vw_tx_active() != 0;
#endif
}

/* Queue a frame.  Returns false without touching the radio when it is busy or
 * the payload is too long, so the caller can simply try again next loop with
 * fresher data. */
inline bool send(const uint8_t *buf, uint8_t len) {
    if (buf == 0 || len == 0 || len > GLOVE_FRAME_LEN) return false;
    if (txBusy()) return false;
#if GLOVE_RF_DRIVER == GLOVE_RF_DRIVER_RH
    if (!radio().send(buf, len)) return false;
    txDeadlineUs() = micros() + frameAirTimeUs(len);
    return true;
#else
    /* vw_send() takes a non-const pointer but does not modify the buffer. */
    return vw_send(const_cast<uint8_t *>(buf), len) != 0;
#endif
}

/* Copy out one received frame whose FCS was good.
 *
 * *len MUST hold the size of buf on entry and comes back holding the number
 * of bytes actually received -- restore it before every call. */
inline bool receive(uint8_t *buf, uint8_t *len) {
    if (buf == 0 || len == 0 || *len == 0) return false;
#if GLOVE_RF_DRIVER == GLOVE_RF_DRIVER_RH
    return radio().recv(buf, len);
#else
    return vw_get_message(buf, len) != 0;
#endif
}

/* Driver-level counters, useful for telling "bad wiring" apart from "bad
 * radio conditions".  Both saturate (8 bits for VirtualWire, 16 for
 * RadioHead), so treat them as trends, not absolutes. */
inline uint32_t rxGood() {
#if GLOVE_RF_DRIVER == GLOVE_RF_DRIVER_RH
    return radio().rxGood();
#else
    return vw_get_rx_good();
#endif
}

inline uint32_t rxBad() {
#if GLOVE_RF_DRIVER == GLOVE_RF_DRIVER_RH
    return radio().rxBad();
#else
    return vw_get_rx_bad();
#endif
}

} /* namespace rflink */

#endif /* RFLINK_H */
