package org.firstinspires.ftc.teamcode;

public class Constants {
    public static final class HardWareConstants {
        public static final class MecanumHardWareConstants{
            public static final String frontLeftMotor = "FrontLeft";
            public static final String frontRightMotor = "FrontRight";
            public static final String backLeftMotor = "BackLeft";
            public static final String backRightMotor = "BackRight";

            public static final String IMU = "imu";
        }

        public static final class OdometryHardwareConstants{
            public static final String Limelight3a = "Limelight3A";
        }
    }

    public static final class OIConstants{

        public static final class MecanumOIConstants{
            public static final double joystickDeadband = 0.05;
        }

    }

    public static final class PoseManagerConstants {
        public static final int limelightPipeline = 0;
        public static final int limlightPollRateHZ = 100;

        public static final long maxStalenessMs = 100;
        public static final int minTagCount = 1;
        public static final double maxSingleTagDistanceMeters = 3.0;
        public static final double maxVisionOdomErrorInches = 12.0;
        public static final double visionWeightXY = 0.15;
        public static final double visionWeightHeading = 0.05;
    }

    public static final class MecanumConstants {
        public static final double thresHold = 0.05;
        public static final double strafeCorrection = 1.1;
        public static boolean isFieldCentric = true;
        public static final double ticksPerInch = 100; //TODO

//        public static final class encoderAutoConstants{
//            public static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // REV ultraplanetary motor
//            public static final double     DRIVE_GEAR_REDUCTION    = 20 ;     // External Gearing.
//            public static final double     WHEEL_DIAMETER_INCHES   = 3 ;     // For figuring circumference
//            public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//                    (WHEEL_DIAMETER_INCHES * Math.PI);
//
//        }
    }
}