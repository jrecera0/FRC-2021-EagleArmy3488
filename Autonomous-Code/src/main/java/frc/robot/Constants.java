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
        public static final int kFrontRightMotorID = 3;
        public static final int kBackLeftMotorID = 1;
        public static final int kBackRightMotorID = 4;

        // Directional Logic
        //public static final boolean kLeftMotorsInverted = true;
        //public static final boolean kRightMotorsInverted = true;
        public static final boolean kIsLeftVoltageInverted = false;
        public static final boolean kIsRightVoltageInverted = true;
        public static final boolean kLeftEncoderReversed = true;
        public static final boolean kRightEncoderReversed = false;
        public static final boolean kGyroReversed = false;

        // Physical Robot Properties
        public static final double kTrackWidth = 24.25;
        public static final double kWheelRadius = 3.625;
        public static final double kGearRatio = 10.75;
        public static final double kEncoderResolution = 2048;

        // Characterization Constants
        public static final double kP = 0.0109;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.226;
        public static final double kV = 1.95;
        public static final double kA = 0.299;
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        // Logging
        public static final boolean kLoggingEnabled = false;
    }

    public class FieldPositioning {
        // Path A RED
        public static final double aRed_tx = 28;
        public static final double aRed_ty = -19;
        // Path A BLUE
        public static final double aBlue_tx = -20.33;
        public static final double aBlue_ty = -12.09;
        // Path B RED
        public static final double bRed_tx = 1.82;
        public static final double bRed_ty = -22.87;
        // Path B Blue
        public static final double bBlue_tx = 3.78;
        public static final double bBlue_ty = -14.01;
         
    }
}
