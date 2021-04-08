// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Challenge.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;

import frc.robot.subsystems.Controller;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  Indexer indexer;
  Intake intake;
  Shooter shooter;
  DriveTrain driveTrain;
  Controller xbox;
  AHRS navx;

  boolean usingTriggerControls;
  double desiredAngle;
  boolean compensating;
  double manualSpeed;
  double increment;

  public void gyroCompensation() {
    if((xbox.getLeftTrigger() >= 0.5)) {
      compensating = true;
    } else {
      desiredAngle = navx.getAngle();
      compensating = false;
    }
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    indexer = new Indexer();
    xbox = new Controller();
    intake = new Intake();
    driveTrain = new DriveTrain();
    shooter = new Shooter();
    navx = new AHRS(SPI.Port.kMXP);
    usingTriggerControls = false;
    compensating = false;
    desiredAngle = 0.0;
    manualSpeed = -0.38;
    increment = 0.1;
  }
  
  public void drive()
  {
    if(!compensating) {
      // leftPower = (frontLeft.getMotorOutputPercent() + backLeft.getMotorOutputPercent()) / 2.0;
      // rightPower = (frontRight.getMotorOutputPercent() + backRight.getMotorOutputPercent()) / 2.0;
      driveTrain.arcadeDrive(-xbox.getLeftStickY(), xbox.getRightStickX()); 
    } else {
      if(navx.getAngle() <= desiredAngle) {
        driveTrain.arcadeDrive(-xbox.getLeftStickY(), xbox.getRightStickX() + increment);
      }
      else if(navx.getAngle() >= desiredAngle) {
        driveTrain.arcadeDrive(-xbox.getLeftStickY(), xbox.getRightStickX() - increment);
      }
    }
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if(kCurrChallenge.equals("ZoneShoot")) {
      // shooty stuff
      if(xbox.getBButton()) {
        shooter.setRedZone();
      }
      if(xbox.getAButton()) {
        shooter.setGreenZone();
      }
      if(xbox.getXButton()) {
        shooter.setBlueZone();
      }
      if(xbox.getYButton()) {
        shooter.setYellowZone();
      }
      if(xbox.getRightBumper()) {
        shooter.shoot();
        if(xbox.getLeftBumper() ) {
          indexer.index();
        } else if (xbox.getRightTrigger() >= 0.5) {
          indexer.moveToShooter();
        } else {
          indexer.stop();
        }
      } else {
        shooter.stop();
        if(xbox.getLeftBumper()) {
          indexer.index();
        } else {
          indexer.stop();
        }
      }
    }
    else if(kCurrChallenge.equals("SpeedShoot")) {
      if(xbox.getRightBumper()) {
        shooter.setSpeedManual(manualSpeed);
        if(xbox.getLeftBumper() ) {
          indexer.index();
        } else if (xbox.getRightTrigger() >= 0.5) {
          indexer.moveToShooter();
        } else {
          indexer.stop();
        }
      } else {
        shooter.stop();
        if(xbox.getLeftBumper()) {
          indexer.index();
        } else {
          indexer.stop();
        }
      }
      if(xbox.getYButton()) {
        manualSpeed-=0.01;
      }
      if(xbox.getAButton()) {
        manualSpeed+=0.01;
      }
      if (xbox.getXButton() ){
        System.out.println("WARNING! manualSpeed: " + manualSpeed);
      }
    }
    drive();
    gyroCompensation();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}
}
