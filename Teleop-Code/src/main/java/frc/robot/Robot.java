// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final int FRONT_LEFT_CAN =2;
  public static final int FRONT_RIGHT_CAN =4;
  public static final int BACK_LEFT_CAN=1;
  public static final int BACK_RIGHT_CAN=3;

  private WPI_TalonFX leftMaster;
  private WPI_TalonFX leftSlave;
  private WPI_TalonFX rightMaster;
  private WPI_TalonFX rightSlave;
  
  private SpeedControllerGroup leftMotors;
  private SpeedControllerGroup rightMotors;
  private DifferentialDrive driveTrain;


  private XboxController xbox;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() 
  {
    leftMaster = new WPI_TalonFX( Robot.FRONT_LEFT_CAN );
    leftSlave = new WPI_TalonFX( Robot.BACK_LEFT_CAN );
    rightMaster = new WPI_TalonFX( Robot.BACK_RIGHT_CAN );
    rightSlave = new WPI_TalonFX( Robot.FRONT_RIGHT_CAN );

    leftMotors = new SpeedControllerGroup( leftMaster, leftSlave );
   
    //leftMotors.setInverted( Robot.LEFT_MOTOR_INVERTED );

    rightMotors = new SpeedControllerGroup( rightMaster, rightSlave );
   
    //rightMotors.setInverted( Robot.RIGHT_MOTOR_INVERTED );

    driveTrain = new DifferentialDrive( leftMotors, rightMotors );

    xbox = new XboxController(0);



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
  public void teleopPeriodic() 
  {
    
    driveTrain.arcadeDrive(xbox.getRawAxis(1), xbox.getRawAxis(0));
  














  }

  private int getRawAxis(int i) {
    return 0;
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() 
  {
  }


    public  double getLeftEncoderPosition() { return leftMaster.getSelectedSensorPosition(); }
    public  double getRightEncoderPosition() { return rightMaster.getSelectedSensorPosition(); }
    public  double getLeftEncoderVelocity() { return leftMaster.getSelectedSensorVelocity(); }
    public  double getRightEncoderVelocity() { return rightMaster.getSelectedSensorVelocity(); }




  
}
