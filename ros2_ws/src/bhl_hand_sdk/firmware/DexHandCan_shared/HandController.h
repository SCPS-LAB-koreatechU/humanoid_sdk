#pragma once
#include <stdint.h>
#include "PoseReceiver.h"
#include "StatusReport.h"

// Hardware-independent receive/output state machine, shared with native tests.
// Output   provides off() and bool apply(const uint16_t pose[15]).
// Reporter provides bool send(uint32_t id, const uint8_t data[8]); it must never
//          block (a full TX queue simply drops the report, PWM is never delayed).
//
// v2 adds replies: one STATUS per pose sequence (applied or rejected), one per
// control command, a HEARTBEAT when nothing else was sent for heartbeatMs, and
// PONG for a PING. Receiving behaviour is unchanged from v1.
template <typename Output, typename Reporter>
class HandController {
 public:
  HandController(Output& output, Reporter& reporter, uint8_t node, uint32_t controlId,
                 uint32_t poseId, const uint16_t low[15], const uint16_t high[15],
                 uint32_t assemblyMs, uint32_t watchdogMs, uint32_t heartbeatMs,
                 const uint8_t profileTag[4])
      : output_(output), reporter_(reporter), node_(node), controlId_(controlId),
        poseId_(poseId), pingId_(hand_status::canId(hand_status::kFunctionPing, node)),
        statusId_(hand_status::canId(hand_status::kFunctionStatus, node)),
        pongId_(hand_status::canId(hand_status::kFunctionPong, node)),
        receiver_(low, high, assemblyMs), watchdogMs_(watchdogMs), heartbeatMs_(heartbeatMs) {
    for (uint8_t i = 0; i < 4; ++i) tag_[i] = profileTag[i];
  }

  void begin() {
    output_.off();
    pwmOn_ = false;
    inhibited_ = false;  // battery boot: first complete valid pose starts output
    invalidatePose();
  }

  void invalidatePose() {
    receiver_.reset();
    haveRecentPose_ = false;
    inhibitedSeqValid_ = false;
  }

  // Set by the CAN adapter; reported in the flags byte, cleared once reported.
  void noteRxOverflow() { sticky_ |= hand_status::RxOverflow; }
  void noteCanRecovered() { sticky_ |= hand_status::CanRecovered; }
  void setErrorCounters(uint32_t tx, uint32_t rx) {
    txErrors_ = tx > 255 ? 255 : static_cast<uint8_t>(tx);
    rxErrors_ = rx > 255 ? 255 : static_cast<uint8_t>(rx);
  }

  // True tells the CAN adapter to discard queued old frames. Retain last PWM.
  bool checkWatchdog(uint32_t now) {
    if (!haveRecentPose_ || static_cast<uint32_t>(now - lastPoseMs_) < watchdogMs_)
      return false;
    invalidatePose();
    watchdogExpired_ = true;
    return true;
  }

  // Call every loop: a HEARTBEAT when nothing was reported for heartbeatMs.
  void service(uint32_t now) {
    if (heartbeatMs_ == 0 || static_cast<uint32_t>(now - lastReportMs_) < heartbeatMs_) return;
    report(0, hand_status::Heartbeat, 0xFF, now);
  }

  void receive(uint32_t id, bool extended, bool remote, uint8_t length,
               const uint8_t* data, uint32_t now) {
    if (extended || remote) return;
    if (id == pingId_ && length >= 1 && data[0] == hand_status::kPingMagic) {
      uint8_t pong[8] = {hand_status::kPingMagic, hand_status::kProtocolVersion, node_, flags(),
                         tag_[0], tag_[1], tag_[2], tag_[3]};
      reporter_.send(pongId_, pong);
      return;
    }
    if (id == controlId_ && length == 2 && data[1] == node_) {
      switch (data[0]) {
        case 0x00:  // OFF: stop PWM, reject new poses
          output_.off();
          pwmOn_ = false;
          inhibited_ = true;
          break;
        case 0x01:  // START: clear output/sequence, permit a new sender session
          output_.off();
          pwmOn_ = false;
          inhibited_ = false;
          break;
        case 0x02:  // HOLD: retain PWM, reject new poses
          inhibited_ = true;
          break;
        case 0x13:  // RESUME: retain PWM, permit a fresh pose stream
          inhibited_ = false;
          break;
        default:
          return;  // unknown mode: no state change, no reply
      }
      invalidatePose();
      report(0, hand_status::ControlAck, data[0], now);
      return;
    }
    if (id != poseId_ || length != 8) return;
    if (inhibited_) {
      // One REJECT_INHIBITED per sequence, so the PC learns it is talking to a held hand.
      if (!inhibitedSeqValid_ || inhibitedSeq_ != data[0]) {
        inhibitedSeqValid_ = true;
        inhibitedSeq_ = data[0];
        report(data[0], hand_status::RejectInhibited, 0xFF, now);
      }
      return;
    }

    uint16_t completePose[15];
    const PoseReceiver::Result result = receiver_.accept(data, now, completePose);
    if (result == PoseReceiver::Result::Rejected) {
      report(receiver_.sequence(), static_cast<uint8_t>(receiver_.reason()), receiver_.channel(), now);
      return;
    }
    if (result != PoseReceiver::Result::Complete) return;
    // Apply immediately on the fifth valid group, without another 20 ms wait.
    // All callbacks run on the same loop, so a reader cannot see a partial set.
    if (!output_.apply(completePose)) {
      output_.off();
      pwmOn_ = false;
      inhibited_ = true;
      const uint8_t sequence = receiver_.sequence();
      invalidatePose();
      report(sequence, hand_status::OutputFail, 0xFF, now);
      return;  // explicit START/RESUME is required after an output failure
    }
    pwmOn_ = true;
    watchdogExpired_ = false;
    ++applied_;
    lastPoseMs_ = now;
    haveRecentPose_ = true;
    report(receiver_.sequence(), hand_status::Applied, 0xFF, now);
  }

  uint16_t appliedCount() const { return applied_; }

 private:
  uint8_t flags() const {
    uint8_t f = sticky_;
    if (pwmOn_) f |= hand_status::PwmOn;
    if (inhibited_) f |= hand_status::Inhibited;
    if (watchdogExpired_) f |= hand_status::WatchdogExpired;
    return f;
  }

  void report(uint8_t sequence, uint8_t result, uint8_t detail, uint32_t now) {
    uint8_t frame[8] = {sequence, result, flags(), detail,
                        static_cast<uint8_t>(applied_ & 0xFF), static_cast<uint8_t>(applied_ >> 8),
                        txErrors_, rxErrors_};
    if (reporter_.send(statusId_, frame)) sticky_ = 0;  // reported; keep them if the frame was dropped
    lastReportMs_ = now;
  }

  Output& output_;
  Reporter& reporter_;
  const uint8_t node_;
  const uint32_t controlId_, poseId_, pingId_, statusId_, pongId_;
  PoseReceiver receiver_;
  const uint32_t watchdogMs_, heartbeatMs_;
  uint8_t tag_[4] = {};
  bool inhibited_ = false;
  bool pwmOn_ = false;
  bool watchdogExpired_ = false;
  bool haveRecentPose_ = false;
  bool inhibitedSeqValid_ = false;
  uint8_t inhibitedSeq_ = 0;
  uint8_t sticky_ = 0;
  uint8_t txErrors_ = 0, rxErrors_ = 0;
  uint16_t applied_ = 0;
  uint32_t lastPoseMs_ = 0;
  uint32_t lastReportMs_ = 0;
};
