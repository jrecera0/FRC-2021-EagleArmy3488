// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  private WPI_TalonFX frontLeft;
  private WPI_TalonFX frontRight;
  private WPI_TalonFX backLeft;
  private WPI_TalonFX backRight;
  private SpeedControllerGroup leftSide;
  private SpeedControllerGroup rightSide;
  private DifferentialDrive driveTrain;
  
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    frontLeft = new WPI_TalonFX(kFrontLeftID);
    frontRight = new WPI_TalonFX(kFrontRightID);
    backLeft = new WPI_TalonFX(kBackLeftID);
    backRight = new WPI_TalonFX(kBackRightID);
    frontLeft.configOpenloopRamp(kRampInSec);
    frontRight.configOpenloopRamp(kRampInSec);
    backLeft.configOpenloopRamp(kRampInSec);
    backRight.configOpenloopRamp(kRampInSec);
    frontLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);
    backLeft.setNeutralMode(NeutralMode.Brake);
    backRight.setNeutralMode(NeutralMode.Brake);
    leftSide = new SpeedControllerGroup(frontLeft, backLeft);
    rightSide = new SpeedControllerGroup(frontRight, backRight);
    driveTrain = new DifferentialDrive(leftSide, rightSide);
  }

  public void arcadeDrive(double fwd, double rot) {
    driveTrain.arcadeDrive(fwd, rot);
  }

  public void tankDrive(double left, double right) {
    driveTrain.tankDrive(left, right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
