// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  private WPI_TalonFX frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
  private SpeedControllerGroup leftGroup, rightGroup;
  private DifferentialDrive driveTrain;

  @Override
  public void robotInit() {
    frontLeftMotor = new WPI_TalonFX(2);
    backLeftMotor = new WPI_TalonFX(1);
    frontRightMotor = new WPI_TalonFX(4);
    backRightMotor = new WPI_TalonFX(3);

    leftGroup = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
    rightGroup = new SpeedControllerGroup(frontRightMotor, backRightMotor);

    driveTrain = new DifferentialDrive(leftGroup, rightGroup);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    
    driveTrain.tankDrive(0.5, 0.494);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
