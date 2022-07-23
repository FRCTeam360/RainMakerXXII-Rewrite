// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final class PneumaticChannels{
        public final static int INTAKE_FORWARD = 0;
        public final static int INTAKE_REVERSE = 1;
    }

    public final class OIPorts{
        public final static int DRIVER_CONT_PORT = 0;
        public final static int OPERATOR_CONT_PORT = 1;
    }
    public final class CANIds{
        public final static int INTAKE_ID = 10;
        public final static int PNEUMATIC_ID = 20;
        public final static int FEEDER_ID = 12;
        public final static int TOWER_ID = 8;
        public final static int MOTOR_L_LEAD_ID = 1;
        public final static int MOTOR_L1_FOLLOW_ID = 2;
        public final static int MOTOR_L2_FOLLOW_ID = 3;
        public final static int MOTOR_R_LEAD_ID = 4;
        public final static int MOTOR_R1_FOLLOW_ID = 5;
        public final static int MOTOR_R2_FOLLOW_ID = 6;
        public final static int FLYWHEEL_LEAD_ID = 7;
        public final static int FLYWHEEL_FOLLOW_ID = 9;
        public final static int CLIMBER_L_ID = 13;
        public final static int CLIMBER_R_ID = 14;
    }
    public static final class DigitalInputPorts {
        public static final int TOP_TOWER = 0;
        public static final int BOTTOM_TOWER = 2;
        public static final int FEEDER = 3;
        public static final int MIDDLE_LIMIT_SWITCH = 1;
    }


}
