#pragma once
#include <cstdint>
#include "pico/time.h"     // absolute_time_t, get_absolute_time, diff/make/timeout
#include "protocol.h"      // rf_protocol::TacticalState

using rf_protocol::TacticalState;

// Silence that separates a normal link blip from real loss. Set well above the
// worst expected gap on the link in use (a few NOP polls on nRF24; seconds on
// an acoustic analog). This is the one knob that defines "loss".
constexpr uint32_t LIVENESS_TIMEOUT_MS  = 750;
constexpr int32_t  FALLBACK_DECEL_MM_S2 = 800;   // bounded decel in (mm/s)/s

// The Wanderer's vehicle state, modeled as an explicit FSM. It owns the
// commanded targets and the arm/stop/fault gating; the radio layer feeds it
// commands and reads its outputs.
class TacticalCore {
public:
    // Liveness: ANY valid frame from ANY commander, regardless of type (NOP
    // included). Fed at the transport boundary, not per command -- "a commander
    // is talking to me" is separate from "what it asked for".
    void note_commander_alive(absolute_time_t now) { last_seen_ = now; }

    void cmd_arm(absolute_time_t now) {
        if (state_ == TacticalState::Safe) enter(TacticalState::Active, now);
    }

    // stop disarms to Safe from either movement-capable state; refused only
    // while a fault is latched (clear it first).
    void cmd_stop(absolute_time_t now) {
        if (state_ != TacticalState::Fault) enter(TacticalState::Safe, now);
    }

    bool cmd_move(int16_t left, int16_t right, absolute_time_t now) {
        // A fresh steering command is the ONLY thing that resumes from Fallback:
        // the commander must re-assert intent, never silently re-accelerate.
        if (state_ == TacticalState::Fallback) enter(TacticalState::Active, now);
        if (state_ != TacticalState::Active) return false;   // Safe / Fault: ignore
        cmd_left_  = left;
        cmd_right_ = right;
        return true;
    }

    void raise_fault(uint16_t code, absolute_time_t now) {
        fault_code_ = code;
        enter(TacticalState::Fault, now);
    }
    bool clear_fault(bool condition_cleared, absolute_time_t now) {
        if (state_ == TacticalState::Fault && condition_cleared) {
            fault_code_ = 0;
            enter(TacticalState::Safe, now);
            return true;
        }
        return false;   // refused while the condition persists
    }

    // Periodic. Drives the liveness check and the Fallback ramp. Call every loop.
    void tick(absolute_time_t now) {
        if (state_ == TacticalState::Active &&
            absolute_time_diff_us(last_seen_, now) >
                static_cast<int64_t>(LIVENESS_TIMEOUT_MS) * 1000) {
            enter(TacticalState::Fallback, now);
        } else if (state_ == TacticalState::Fallback) {
            ramp_toward_zero(now);
        }
        last_tick_ = now;
    }

    // --- the FSM's only outputs; the motor/control layer consumes these ---
    bool motors_enabled() const {
        return state_ == TacticalState::Active || state_ == TacticalState::Fallback;
    }
    int16_t target_left()  const { return output(cmd_left_,  ramp_left_);  }
    int16_t target_right() const { return output(cmd_right_, ramp_right_); }
    TacticalState state() const { return state_; }

private:
    void enter(TacticalState next, absolute_time_t now) {
        if (next == state_) return;
        if (next == TacticalState::Fallback) {       // seed the ramp from the standing order
            ramp_left_  = cmd_left_;
            ramp_right_ = cmd_right_;
        }
        if (next == TacticalState::Safe || next == TacticalState::Fault) {
            cmd_left_ = cmd_right_ = ramp_left_ = ramp_right_ = 0;
        }
        state_     = next;
        last_tick_ = now;
        on_transition(next);                          // event seam (see below)
    }

    int16_t output(int16_t cmd, int16_t ramp) const {
        if (state_ == TacticalState::Active)   return cmd;
        if (state_ == TacticalState::Fallback) return ramp;
        return 0;                                     // Safe / Fault gate to zero
    }

    void ramp_toward_zero(absolute_time_t now) {
        const int64_t dt_us = absolute_time_diff_us(last_tick_, now);
        const int32_t step  = static_cast<int32_t>(
            static_cast<int64_t>(FALLBACK_DECEL_MM_S2) * dt_us / 1000000);
        ramp_left_  = decay(ramp_left_,  step);
        ramp_right_ = decay(ramp_right_, step);
    }
    static int16_t decay(int16_t v, int32_t step) {
        if (v > 0) return static_cast<int16_t>(v >  step ? v - step : 0);
        if (v < 0) return static_cast<int16_t>(-v > step ? v + step : 0);
        return 0;
    }

    void on_transition(TacticalState) {}   // stage one: log; later: push an event frame

    TacticalState   state_     = TacticalState::Safe;
    int16_t cmd_left_  = 0, cmd_right_  = 0;   // standing order
    int16_t ramp_left_ = 0, ramp_right_ = 0;   // Fallback ramp output
    uint16_t fault_code_ = 0;
    absolute_time_t last_seen_ = {};           // liveness clock
    absolute_time_t last_tick_ = {};
};