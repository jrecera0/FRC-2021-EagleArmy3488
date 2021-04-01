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
    public class DriveConstants {
        // CANBus
        public static final int kFrontLeftMotorID = 2;
        public static final int kFrontRightMotorID = 4;
        public static final int kBackLeftMotorID = 1;
        public static final int kBackRightMotorID = 3;

        // Directional Logic
        //public static final boolean kLeftMotorsInverted = true;
        //public static final boolean kRightMotorsInverted = true;
        public static final boolean kIsLeftVoltageInverted = false; // OLD false
        public static final boolean kIsRightVoltageInverted = true; // OLD true
        public static final boolean kLeftEncoderReversed = false; // OLD false
        public static final boolean kRightEncoderReversed = true; // OLD true
        public static final boolean kGyroReversed = false; // OLD false


        // Physical Robot Properties
        public static final double kTrackWidth = 24.3125;
        public static final double kWheelRadius = 3.425;
        public static final double kGearRatio = 10.71;
        public static final double kEncoderResolution = 2048;

        // Characterization Constants (ACTUAL)
        // public static final double kP = 0.0109;
        // public static final double kI = 0.0;
        // public static final double kD = 0.0;
        // public static final double kS = 0.226;
        // public static final double kV = 1.95;
        // public static final double kA = 0.299;
        // TESTING
        public static final double kP = 2.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.68;
        public static final double kV = 1.95;
        public static final double kA = 0.242;
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        // Max Speeds
        public static final double kVelocityMax = 2; //original: 1.8288; // 6ft/s
        public static final double kAccelerationMax = 1.5; //original: 1.2192; // 4ft/s^2
        // Logging
        public static final boolean kLoggingEnabled = false; // do. not. set. true.
    }

    public class FieldPositioning {
        // Other
        public static final double kInterval = 2;
        // Path A RED
        public static final double aRed_tx = 58.60;
        public static final double aRed_ty = -49.30;
        // Path A BLUE
        public static final double aBlue_tx = 47.65;
        public static final double aBlue_ty = -45.5;
        // Path B RED
        public static final double bRed_tx = 38.21;
        public static final double bRed_ty = -52.15;
        // Path B Blue
        public static final double bBlue_tx = 35.75;
        public static final double bBlue_ty = -44.50;
    }

    public class BlinkinConstants {
        public static final double kRed = 0.61;
        public static final double kOrange = 0.65;
        public static final double kYellow = 0.69;
        public static final double kGreen = 0.77;
        public static final double kBlue = 0.79;
        public static final double kViolet = 0.91;
        public static final double kWhite = 0.93;
        public static final double kBlack = 0.99;        
        public static final int kBlinkinPort = 0;
    }
        
    public class TrajectoryPathnames {
        public static final String kBarrelRacePath = "paths/BarrelRace.wpilib.json";
        public static final String kBouncePath = "paths/bounce.wpilib.json";
        public static final String kLinePath = "paths/Bouce.wpilib.json";
    }
}
