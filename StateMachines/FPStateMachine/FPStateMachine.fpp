module Billee {
    @ Per-subsystem fault latch.
    @
    @ Detection (thresholds, debounce, undervoltage gating) lives in FPManager's C++; this state
    @ machine only latches. Once FAULTED it stays FAULTED (further faults are ignored, so there
    @ is no re-trip spam and no auto-clear) until the ground sends fpManager.CLEAR_FAULT.
    state machine FPStateMachine {

        @ Enter NOMINAL on component startup: no fault latched yet
        initial enter NOMINAL

        @ A debounced fault was detected on this subsystem
        signal fault: Billee.FaultReason

        @ Ground requested a clear (FPManager has already checked it is allowed)
        signal clearRequest

        @ Latch the fault: inhibit the subsystem (if controllable) and log why
        action doTrip: Billee.FaultReason

        @ Release the latch: reset detection state and release the inhibit
        action doClear

        state NOMINAL {
            on fault do {doTrip} enter FAULTED
        }

        state FAULTED {
            on clearRequest do {doClear} enter NOMINAL
        }
    }
}
