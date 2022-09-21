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
        public final static int TURRET_ID = 11;
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
        public final static int PIGEON_ID = 22;
    }

    public static final class DigitalInputPorts {
        public static final int TOP_TOWER = 0;
        public static final int BOTTOM_TOWER = 2;
        public static final int FEEDER = 3;
        public static final int MIDDLE_LIMIT_SWITCH = 1;
    }

    public static final class DriveTrainConstants {
        public static final double trackWidthMeters = 0.641;
        public static final double ticksToMeters = ((Math.PI * 0.1524) * ((15.0 / 85.0) * (24.0 / 46.0) / 2048.0));
    }

    public static final class FieldConstants {
        public static final double hubX = 8;
        public static final double hubY = 5;
    }

    public static final class AutoConstants {

        private static final double xOffsetTerminal5Ball = -0.33 - 0.1;
        private static final double yOffsetTerminal5Ball = 0.22 + 0.1;
        private static final double xOffsetHangarEmergency = 0;
        private static final double yOffsetHangarEmergency = 0;
        private static final double xOffsetHangar4Ball = 0;
        private static final double yOffsetHangar4Ball = 0;

        public static double getXOffsetTerminal5Ball() {
            if(fieldType == FieldType.COMP) {
                return xOffsetTerminal5Ball;
            }
            return 0.0;
        }

        public static double getYOffsetTerminal5Ball() {
            if(fieldType == FieldType.COMP) {
                return yOffsetTerminal5Ball;
            }
            return 0.0;
        }

        public static double getXOffsetHangarEmergency() {
          if(fieldType == FieldType.COMP) {
              return xOffsetHangarEmergency;
          }
          return 0.0;
      }

      public static double getYOffsetHangarEmergency() {
          if(fieldType == FieldType.COMP) {
              return yOffsetHangarEmergency;
          }
          return 0.0;
      }

        public static double getXOffsetHangar4Ball() {
          if(fieldType == FieldType.COMP) {
              return xOffsetHangar4Ball;
          }
          return 0.0;
      }

      public static double getYOffsetHangar4Ball() {
          if(fieldType == FieldType.COMP) {
              return yOffsetHangar4Ball;
          }
          return 0.0;
      }

        //values for Ramsete controller
        public static final double ksVolts = 0.59619;
        public static final double kvVoltSecondsPerMeter = 1.2895;
        public static final double kaVoltSecondsSquaredPerMeter = 0.16441;
        public static final double kPDriveVel = 1.7177;
        public static final double kTrackwidthMeters = 0.641;

        public static final double kMaxSpeedMetersPerSecondHigh = 2.0;
        public static final double kMaxAccelerationMetersPerSecondSquaredHigh = 2.0;

        public static final double kMaxSpeedMetersPerSecondLow = 1.0;
        public static final double kMaxAccelerationMetersPerSecondSquaredLow = 1.0;

        //ramsete values - 2,.7 are default
        public static final double kRamseteB = 2.0;
        public static final double kRamseteZeta = 0.7;

        public static final boolean kGyroReversed = true; //Characterization says this isn't necessary but it seems to perform better....
    }

    public enum FieldType {
        COMP, PRACTICE
    }

    public enum RobotType {
        COMP, PRACTICE
    }

    public static final FieldType fieldType = FieldType.COMP;
    
    public static final RobotType robotType = RobotType.COMP;

    public static FieldType getFieldType(){
        return fieldType;
    }

}
