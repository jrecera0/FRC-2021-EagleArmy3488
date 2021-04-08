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
    public class Challenge {
        // Valid options:
        // AutoNav - "AutoNav"
        // Galatic Search - "GSearch"
        public static final String kCurrChallenge = "GSearch";
    }

    public class TrajectoryPathnames {
        // Specific Pathnames
        public static final String kBarrelRacePath = "paths/BarrelRace.wpilib.json";
        public static final String kBouncePath = "paths/Bounce.wpilib.json";
        public static final String kSlalomPath = "paths/Slalom.wpilib.json";
        public static final String kTestPath = "paths/Test.wpilib.json"; // Change as needed

        // Jank 100
        public static final String kSlalomPathnamePt1 = "paths/SlalomSect1.wpilib.json";
        public static final String kSlalomPathnamePt2 = "paths/SlalomSect2.wpilib.json";
        public static final String kSlalomPathnamePt3 = "paths/SlalomSect3.wpilib.json";
        public static final String kSlalomPathnamePt4 = "paths/SlalomSect4.wpilib.json";

        // Just so we can select a path to run
        // Modes: "Bounce", "Barrel", "Slalom", "Test", "SlalomIsSpecial"
        // Bounce is Slalom, Slalom is Bounce
        public static final String kCurrentTraj = "Test";
    }

    public class FieldPositioning {
        // Other
        public static final double kInterval = 5;
        // Path A RED
        public static final double aRed_tx = 31.5;
        public static final double aRed_ty = -41.0;
        // Path A BLUE
        public static final double aBlue_tx = 52.9;
        public static final double aBlue_ty = -33.5;
        // Path B RED
        public static final double bRed_tx = -10.8;
        public static final double bRed_ty = -44.0;
        // Path B Blue
        public static final double bBlue_tx = 43.25;
        public static final double bBlue_ty = -33.50;
    }

    public class DriveConstants {
        // CANBus
        public static final int kFrontLeftMotorID = 1;
        public static final int kBackLeftMotorID = 2;
        public static final int kFrontRightMotorID = 3;
        public static final int kBackRightMotorID = 4;

        // Directional Logic
        //public static final boolean kLeftMotorsInverted = true;
        //public static final boolean kRightMotorsInverted = true;
        public static final boolean kIsLeftVoltageInverted = false; // OLD false
        public static final boolean kIsRightVoltageInverted = true; // OLD true
        public static final boolean kLeftEncoderReversed = false; // OLD false
        public static final boolean kRightEncoderReversed = true; // OLD true
        public static final boolean kGyroReversed = true; // OLD false


        // Physical Robot Properties
        public static final double kTrackWidth = 22.875;
        public static final double kWheelRadius = 2.9;
        public static final double kGearRatio = 10.71;
        public static final double kEncoderResolution = 2048;

        // Characterization Constants (ACTUAL)
        public static final double kP = 2.42;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.629;
        public static final double kV = 2.42;
        public static final double kA = 0.343;
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        // Max Speeds
        public static final double kVelocityMax = 1.8288; // 2 //original: 1.8288; // 6ft/s
        public static final double kAccelerationMax = 1.2192; // 1.5 //original: 1.2192; // 4ft/s^2
        public static final double kSpecialVelocityMax = 5;
        public static final double kSpecialAccelerationMax = 5;
        // Logging
        public static final boolean kLoggingEnabled = false; // do. not. set. true.
    }

    public class IntakeConstants {
        public static final int kIntakeID = 10;
        public static final double kIntakeSpeed = -1.0;
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
}
