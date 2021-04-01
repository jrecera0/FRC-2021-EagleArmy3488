package frc.robot;

public class Constants {
    public class IndexerConstants {
        // CAN IDs
        public static final int kFrontMotorID = 9; // This should be closest to the shooter
        public static final int kMidMotorID = 8;
        public static final int kBackMotorID = 7;
        public static final int kFrontSensorID = 13;
        public static final int kMidSensorID = 12;
        public static final int kBackSensorID = 11;

        // Speeds
        public static final double kIndexerSpeed = 0.2;
        
        // Threshing
        public static final double kSensorThresh = 5.90551; // is in inches, ~150mm
    }
}