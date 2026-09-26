module Billee{
    enum Subsystems: U8 {
        DRIVETRAIN = 1 @< Drivetrain subsystem
        ARM = 2 @< Arm subsystem
        AUX = 3 @< Unused: there is no switched AUX channel in hardware (+12V_AUX_VCC is always on). Kept only so enum values don't renumber.
        SCIENCE = 4 @< Science subsystem
        LOGIC = 5 @< Logic/flight-computer board (monitoring only, no power control - it runs FPManager)
    }
}
