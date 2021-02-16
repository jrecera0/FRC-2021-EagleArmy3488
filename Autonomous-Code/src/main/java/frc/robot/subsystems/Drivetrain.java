// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.util.Units;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

public class Drivetrain extends SubsystemBase {
  public static final int CAN_FRONT_LEFT = 2;
  public static final int CAN_BACK_LEFT = 1;
  public static final int CAN_FRONT_RIGHT = 4;
  public static final int CAN_BACK_RIGHT = 3;

  // track width in inches
  public static final double TRACK_WIDTH = 24.3125;
  // wheel radius in inches
  public static final double WHEEL_RADIUS = 3.625;

  public static final double kS = 0.226;
  public static final double kV = 1.95;
  public static final double kA = 0.299;
  // Velocity
  public static final double kP = 0.0109; // 0.782
  public static final double kD = 0.0;
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;

  private static final double K_GEAR_RATIO = 10.71;
  //private static final double K_GEAR_RATIO = 12.75; // Apparently this is the corrected number, added 1/12/21
  private static final int K_SENSOR_UNITS_PER_ROTATION = 2048;
  private static final int K_WHEEL_UNITS_PER_REVOLUTION = 22016;
  private static final double PULSE_PER_INCH = (K_SENSOR_UNITS_PER_ROTATION / (2 * WHEEL_RADIUS * Math.PI / K_GEAR_RATIO ));
  // NOTE:  LEFT MOTORS - POSITIVE, RIGHT MOTORS - NEGATIVE
  //        WHEN DRIVING FORWARD

  private WPI_TalonFX frontLeftMotor;
  private WPI_TalonFX backLeftMotor;
  private WPI_TalonFX frontRightMotor;
  private WPI_TalonFX backRightMotor;

  private SpeedControllerGroup leftMotors;
  private SpeedControllerGroup rightMotors;

  private DifferentialDrive driveTrain;

  private DifferentialDriveKinematics kinematics;
  private DifferentialDriveOdometry odometry;
  private SimpleMotorFeedforward feedForward;
  
  private PIDController leftPIDController;
  private PIDController rightPIDController;

  private AHRS gyro;

  public Drivetrain()
  {
    frontLeftMotor = new WPI_TalonFX(Drivetrain.CAN_FRONT_LEFT);
    backLeftMotor = new WPI_TalonFX(Drivetrain.CAN_BACK_LEFT);
    frontRightMotor = new WPI_TalonFX(Drivetrain.CAN_FRONT_RIGHT);
    backRightMotor = new WPI_TalonFX(Drivetrain.CAN_BACK_RIGHT);

    
    frontLeftMotor.setNeutralMode( NeutralMode.Brake );
    backLeftMotor.setNeutralMode( NeutralMode.Brake );
    frontRightMotor.setNeutralMode( NeutralMode.Brake );
    backRightMotor.setNeutralMode( NeutralMode.Brake );
    

    leftMotors = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
    rightMotors = new SpeedControllerGroup(frontRightMotor, backRightMotor);

    driveTrain = new DifferentialDrive(leftMotors, rightMotors);

    gyro = new AHRS( SPI.Port.kMXP );
    System.out.println( gyro.getAngle() );
    
    resetEncoders();
    
    kinematics = new DifferentialDriveKinematics( Units.inchesToMeters( Drivetrain.TRACK_WIDTH ) );
    odometry = new DifferentialDriveOdometry( getHeading() );        
    feedForward = new SimpleMotorFeedforward( Drivetrain.kS, Drivetrain.kV, Drivetrain.kA );
    leftPIDController = new PIDController( Drivetrain.kP, 0, Drivetrain.kD );
    rightPIDController = new PIDController( Drivetrain.kP, 0, Drivetrain.kD );
  }

  public void arcadeDrive(double xSpeed, double zRotation) {
    driveTrain.arcadeDrive(xSpeed, zRotation);
  }

  @Override
  public void periodic()
  {
      odometry.update( getHeading(), getLeftWheelDistance(), getRightWheelDistance() );
  }

  public DifferentialDriveKinematics getKinematics() { return kinematics; }
  public DifferentialDriveOdometry getOdometry() { return odometry; }
  public SimpleMotorFeedforward getSimpleMotorFeedForward() { return feedForward; }
  public PIDController getLeftPIDCOntroller() { return leftPIDController; }
  public PIDController getRightPIDCOntroller() { return rightPIDController; }

  public void resetOdometry(Pose2d pose) 
  {
    resetEncoders();
    resetGyro();
    odometry.resetPosition( pose, getHeading() );
  }

  public Pose2d getPose() 
  {
    return odometry.getPoseMeters();
  }

  public void tankDriveVolts( double leftVolts, double rightVolts )
  {
    System.out.println( "WARNING! Position: " + getLeftWheelDistance() + " " + getRightWheelDistance() );
    leftMotors.setVoltage( -leftVolts );
    rightMotors.setVoltage( rightVolts * 0.988 );
    driveTrain.feed();
  }

  public void setMaxOutput( double maxOutput )
  {
    driveTrain.setMaxOutput( maxOutput );
  }

  public void resetEncoders() { frontLeftMotor.setSelectedSensorPosition( 0 ); frontRightMotor.setSelectedSensorPosition( 0 ); }
  public double getLeftEncoderPosition() { return -frontLeftMotor.getSelectedSensorPosition(); }
  public double getRightEncoderPosition() { return frontRightMotor.getSelectedSensorPosition(); }
  public double getLeftEncoderVelocity() { return -frontLeftMotor.getSelectedSensorVelocity(); }
  public double getRightEncoderVelocity() { return frontRightMotor.getSelectedSensorVelocity(); }
  
  public DifferentialDriveWheelSpeeds getSpeeds() 
  {
    // returns wheel speeds in meters per second
    //double leftVelocity = Units.inchesToMeters( getLeftEncoderVelocity() / PULSE_PER_INCH );
    //double rightVelocity = Units.inchesToMeters( getLeftEncoderVelocity() / PULSE_PER_INCH );
    double leftRotationsPerSecond = (double) getLeftEncoderVelocity() / K_SENSOR_UNITS_PER_ROTATION / K_GEAR_RATIO * 10;        
    double leftVelocity = leftRotationsPerSecond * 2 * Math.PI * Units.inchesToMeters( WHEEL_RADIUS );
    double rightRotationsPerSecond = (double) getRightEncoderVelocity() / K_SENSOR_UNITS_PER_ROTATION / K_GEAR_RATIO * 10;        
    double rightVelocity = rightRotationsPerSecond * 2 * Math.PI * Units.inchesToMeters( WHEEL_RADIUS );
    
    System.out.println( "WARNING! Velocity: " + leftVelocity + " " + rightVelocity );

    return new DifferentialDriveWheelSpeeds( leftVelocity, rightVelocity ); 
  }

  // Distance returned in meters
  public double getLeftWheelDistance()
  {
    double leftDistance = ((double) getLeftEncoderPosition()) / K_SENSOR_UNITS_PER_ROTATION / K_GEAR_RATIO * 2 * Math.PI * Units.inchesToMeters( WHEEL_RADIUS );
    return leftDistance;
  }

  // Distance returned in meters
  public double getRightWheelDistance()
  {
    double rightDistance = ((double) getRightEncoderPosition()) / K_SENSOR_UNITS_PER_ROTATION / K_GEAR_RATIO * 2 * Math.PI * Units.inchesToMeters( WHEEL_RADIUS );
    return rightDistance;
  }
  
  public void resetGyro() { gyro.reset(); }
  public double getGyroAngle() { return -gyro.getAngle(); }
  public Rotation2d getHeading() { return Rotation2d.fromDegrees( -gyro.getAngle() ); }
      
  public void printEncoders()
  {
    System.out.print( "WARNING! LEFT Position: " + frontLeftMotor.getSelectedSensorPosition() );
    System.out.print( "WARNING! Velocity: " + frontLeftMotor.getSelectedSensorVelocity() );
    System.out.println();
    System.out.print( "WARNING! RIGHT Position: " + frontRightMotor.getSelectedSensorPosition() );
    System.out.print( "WARNING! Velocity: " + frontRightMotor.getSelectedSensorVelocity() );
    System.out.println();
    System.out.println( "----------------------------------------");
  }   
}
