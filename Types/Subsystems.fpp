module Billee{
    enum Subsystems: U8 {
        DRIVETRAIN = 1 @< Drivetrain subsystem
        ARM = 2 @< Arm subsystem
        AUX = 3 @< Auxiliary subsystem
        SCIENCE = 4 @< Science subsystem
        LOGIC = 5 @< Logic/flight-computer board (monitoring only, no power control - it runs FPManager)
    }
}