package frc.robot;

public class Constants {
    public class Challenge {
        // Valid options:
        // Interstellar Accuracy Challenge - "ZoneShoot"
        // Power Port Challenge - "SpeedShoot"
        public static final String kCurrChallenge = "ZoneShoot";
    }
    public class DriveConstants {
        // CAN IDs
        public static final int kFrontLeftID = 1;
        public static final int kBackLeftID = 2;
        public static final int kFrontRightID = 3;
        public static final int kBackRightID = 4;

        // Ramp!
        public static final double kRampInSec = 0.1875;
    }
    
    public class ShooterConstants {
        // CAN IDs
        public static final int kLeftShooterID = 5;
        public static final int kRightShooterID = 6;

        // Speeds
        public static final double kDefaultSpeed = -0.30;
        // Note about zones: Green is closest, Red is farthest
        public static final double kGreenZoneSpeed = -0.15;
        public static final double kYellowZoneSpeed = -0.30;
        public static final double kBlueZoneSpeed = -0.28;
        public static final double kRedZoneSpeed = -0.28;
    }

    public class IndexerConstants {
        // CAN IDs
        public static final int kFrontMotorID = 9;
        public static final int kMidMotorID = 8;
        public static final int kBackMotorID = 7;
        public static final int kFrontSensorID = 13;
        public static final int kMidSensorID = 12;
        public static final int kBackSensorID = 11;

        // Speeds
        public static final double kIndexerSpeed = 0.18;
        
        // Threshing
        public static final double kSensorThresh = 200.0; // is in mm
    }

    public class IntakeConstants {
        // CAN IDs
        public static final int kIntakeID = 10;

        // Speeds
        public static final double kIntakeSpeed = -1.0;
    }

    public class ControllerConstants {
        public static final int kControllerPort = 0;
        public static final double kDeadzoneVal = 0.02; // Def for DiffDrive
    }
}