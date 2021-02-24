// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private AHRS gyro;
  private DifferentialDrive driveTrain;
  private DifferentialDriveKinematics kinematics;
  private DifferentialDriveOdometry odometry;
  private PIDController leftPIDController, rightPIDController;
  private SimpleMotorFeedforward feedForward;
  private SpeedControllerGroup leftMotorGroup, rightMotorGroup;
  private WPI_TalonFX frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
  
  public DriveTrain() {
    frontLeftMotor = new WPI_TalonFX(kFrontLeftMotorID);
    frontRightMotor = new WPI_TalonFX(kFrontRightMotorID);
    backLeftMotor = new WPI_TalonFX(kBackLeftMotorID);
    backRightMotor = new WPI_TalonFX(kBackRightMotorID);

    setDriveMode(NeutralMode.Brake);

    leftMotorGroup = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
    rightMotorGroup = new SpeedControllerGroup(frontRightMotor, backRightMotor);

    driveTrain = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

    gyro = new AHRS(SPI.Port.kMXP);

    resetEncoders(); // make sure odometry isn't screwed up

    kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(kTrackWidth));
    odometry = new DifferentialDriveOdometry(getHeading());
    feedForward = new SimpleMotorFeedforward(kS, kP, kA);
    leftPIDController = new PIDController(kP, kI, kD);
    rightPIDController = new PIDController(kP, kI, kD);
  }

  @Override
  public void periodic() {
    odometry.update(getHeading(), getLeftWheelDistance(), getRightWheelDistance());
    if(kLoggingEnabled) {
      outputTelementary();
    }
  }

  // Feeds for autonomous


  // Controlling DriveTrain  
  public void arcadeDrive(double fwd, double rot) {
    driveTrain.arcadeDrive(fwd, rot);
  }

  public void setTankDriveVolts(double leftVolts, double rightVolts) {
    leftMotorGroup.setVoltage((kIsLeftVoltageInverted) ? -leftVolts : leftVolts);
    rightMotorGroup.setVoltage((kIsRightVoltageInverted) ? -rightVolts : rightVolts);
    driveTrain.feed();
  }

  public void setMaxOutput(double maxOutput) {
    driveTrain.setMaxOutput(maxOutput);
  }

  public void setDriveMode(NeutralMode driveMode) {
    frontLeftMotor.setNeutralMode(driveMode);
    frontRightMotor.setNeutralMode(driveMode);
    backLeftMotor.setNeutralMode(driveMode);
    backRightMotor.setNeutralMode(driveMode);
  }

  // Obtaining DriveTrain Information
  public DifferentialDriveWheelSpeeds getDriveWheelSpeeds() {
    // returns wheel speeds in m/s (meters per second)
    double leftRotationsPerSecond = (double) getLeftEncoderVelocity() / kEncoderResolution / kGearRatio * 10;        
    double leftVelocity = leftRotationsPerSecond * 2 * Math.PI * Units.inchesToMeters(kWheelRadius);
    double rightRotationsPerSecond = (double) getRightEncoderVelocity() / kEncoderResolution / kGearRatio * 10;        
    double rightVelocity = rightRotationsPerSecond * 2 * Math.PI * Units.inchesToMeters(kWheelRadius);
    return new DifferentialDriveWheelSpeeds(leftVelocity, rightVelocity);
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees((kGyroReversed) ? -gyro.getAngle() : gyro.getAngle());
  }

  public double getLeftWheelDistance() {
    double leftDistance = ((double) getLeftEncoderPosition()) / kEncoderResolution / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadius);
    return leftDistance;
  }

  public double getRightWheelDistance() {
    double rightDistance = ((double) getRightEncoderPosition()) / kEncoderResolution / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadius);
    return rightDistance;
  }

  public double getLeftEncoderPosition() {
    return (kLeftEncoderReversed) ? -frontLeftMotor.getSelectedSensorPosition() : frontLeftMotor.getSelectedSensorPosition();
  }

  public double getRightEncoderPosition() {
    return (kRightEncoderReversed) ? -frontRightMotor.getSelectedSensorPosition() : frontRightMotor.getSelectedSensorPosition();
  }

  public double getLeftEncoderVelocity() {
    return (kLeftEncoderReversed) ? -frontLeftMotor.getSelectedSensorVelocity() : frontLeftMotor.getSelectedSensorVelocity();
  }

  public double getRightEncoderVelocity() {
    return (kRightEncoderReversed) ? -frontRightMotor.getSelectedSensorVelocity() : frontRightMotor.getSelectedSensorVelocity();
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public DifferentialDriveOdometry getOdometry() {
    return odometry;
  }

  public SimpleMotorFeedforward getSimpleMotorFeedForward() {
    return feedForward;
  }

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public double getGyroAngle() {
    return gyro.getAngle();
  }

  // Reset Functions
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    resetGyro();
    odometry.resetPosition(pose, getHeading());
  }

  public void resetEncoders() {
    frontLeftMotor.setSelectedSensorPosition(0);
    frontRightMotor.setSelectedSensorPosition(0);
  }

  public void resetGyro() {
    gyro.reset();
  }

  private void outputTelementary() {
    // note that odometry is in meters and such
    double gyroAngle = (kGyroReversed) ? -gyro.getAngle() : gyro.getAngle();
    double leftSideRotationsPerSecond = (double) getLeftEncoderVelocity() / kEncoderResolution / kGearRatio * 10;
    double leftSidePhysicalVelocity = leftSideRotationsPerSecond * 2 * Math.PI * Units.inchesToMeters(kWheelRadius);
    double leftSideDistanceTraversed = ((double) getLeftEncoderPosition()) / kEncoderResolution / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadius);
    double rawLeftEncoderPosition = (kLeftEncoderReversed) ? -frontLeftMotor.getSelectedSensorPosition() : frontLeftMotor.getSelectedSensorPosition();
    double rawLeftEncoderVelocity = (kLeftEncoderReversed) ? -frontLeftMotor.getSelectedSensorVelocity() : frontLeftMotor.getSelectedSensorVelocity();
    double frontLeftMotorInputCurrent = frontLeftMotor.getSupplyCurrent();
    double frontLeftMotorOutputCurrent = frontLeftMotor.getStatorCurrent();
    double frontLeftMotorOutputVoltage = frontLeftMotor.getMotorOutputVoltage();
    double frontLeftMotorOutputPercent = frontLeftMotor.getMotorOutputPercent();
    double backLeftMotorInputCurrent = backLeftMotor.getSupplyCurrent();
    double backLeftMotorOutputCurrent = backLeftMotor.getStatorCurrent();
    double backLeftMotorOutputVoltage = backLeftMotor.getMotorOutputVoltage();
    double backLeftMotorOutputPercent = backLeftMotor.getMotorOutputPercent();
    double rightSideRotationsPerSecond = (double) getRightEncoderVelocity() / kEncoderResolution / kGearRatio * 10;
    double rightSidePhysicalVelocity = rightSideRotationsPerSecond * 2 * Math.PI * Units.inchesToMeters(kWheelRadius);
    double rightSideDistanceTraversed = ((double) getRightEncoderPosition()) / kEncoderResolution / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadius);
    double rawRightEncoderPosition = (kRightEncoderReversed) ? -frontRightMotor.getSelectedSensorPosition() : frontRightMotor.getSelectedSensorPosition();
    double rawRightEncoderVelocity = (kRightEncoderReversed) ? -frontRightMotor.getSelectedSensorVelocity() : frontRightMotor.getSelectedSensorVelocity();
    double frontRightMotorInputCurrent = frontRightMotor.getSupplyCurrent();
    double frontRightMotorOutputCurrent = frontRightMotor.getStatorCurrent();
    double frontRightMotorOutputVoltage = frontRightMotor.getMotorOutputVoltage();
    double frontRightMotorOutputPercent = frontRightMotor.getMotorOutputPercent();
    double backRightMotorInputCurrent = backRightMotor.getSupplyCurrent();
    double backRightMotorOutputCurrent = backRightMotor.getStatorCurrent();
    double backRightMotorOutputVoltage = backRightMotor.getMotorOutputVoltage();
    double backRightMotorOutputPercent = backRightMotor.getMotorOutputPercent();

    // Verbose logging (the WARNING! allows for info to be seen in driver station)
    System.out.println("WARNING! ====BEGIN DRIVETRAIN LOG====");
    System.out.println("WARNING! gyroAngle: " + gyroAngle);
    System.out.println("WARNING! <<<<<< LEFT TRAIN <<<<<<");
    System.out.println("WARNING! leftSideRotationsPerSecond: " + leftSideRotationsPerSecond);
    System.out.println("WARNING! leftSidePhysicalVelocity: " + leftSidePhysicalVelocity);
    System.out.println("WARNING! leftSideDistanceTraversed: " + leftSideDistanceTraversed);
    System.out.println("WARNING! rawLeftEncoderPosition: " + rawLeftEncoderPosition);
    System.out.println("WARNING! rawLeftEncoderVelocity: " + rawLeftEncoderVelocity);
    System.out.println("WARNING! frontLeftMotorInputCurrent: " + frontLeftMotorInputCurrent);
    System.out.println("WARNING! frontLeftMotorOutputCurrent: " + frontLeftMotorOutputCurrent);
    System.out.println("WARNING! frontLeftMotorOutputVoltage: " + frontLeftMotorOutputVoltage);
    System.out.println("WARNING! frontLeftMotorOutputPercent: " + frontLeftMotorOutputPercent);
    System.out.println("WARNING! backLeftMotorInputCurrent: " + backLeftMotorInputCurrent);
    System.out.println("WARNING! backLeftMotorOutputCurrent: " + backLeftMotorOutputCurrent);
    System.out.println("WARNING! backLeftMotorOutputVoltage: " + backLeftMotorOutputVoltage);
    System.out.println("WARNING! backLeftMotorOutputPercent: " + backLeftMotorOutputPercent);
    System.out.println("WARNING! >>>>>> RIGHT TRAIN >>>>>>");
    System.out.println("WARNING! rightSideRotationsPerSecond: " + rightSideRotationsPerSecond);
    System.out.println("WARNING! rightSidePhysicalVelocity: " + rightSidePhysicalVelocity);
    System.out.println("WARNING! rightSideDistanceTraversed: " + rightSideDistanceTraversed);
    System.out.println("WARNING! rawRightEncoderPosition: " + rawRightEncoderPosition);
    System.out.println("WARNING! rawRightEncoderVelocity: " + rawRightEncoderVelocity);
    System.out.println("WARNING! frontRightMotorInputCurrent: " + frontRightMotorInputCurrent);
    System.out.println("WARNING! frontRightMotorOutputCurrent: " + frontRightMotorOutputCurrent);
    System.out.println("WARNING! frontRightMotorOutputVoltage: " + frontRightMotorOutputVoltage);
    System.out.println("WARNING! frontRightMotorOutputPercent: " + frontRightMotorOutputPercent);
    System.out.println("WARNING! backRightMotorInputCurrent: " + backRightMotorInputCurrent);
    System.out.println("WARNING! backRightMotorOutputCurrent: " + backRightMotorOutputCurrent);
    System.out.println("WARNING! backRightMotorOutputVoltage: " + backRightMotorOutputVoltage);
    System.out.println("WARNING! backRightMotorOutputPercent: " + backRightMotorOutputPercent);
  }
}
