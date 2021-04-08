// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // private static final String kDefaultAuto = "Default";
  // private static final String kCustomAuto = "My Auto";
  // private String m_autoSelected;
  // private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // can id things
  private int frontLeft_CAN = 1;
  private int backLeft_CAN = 2;
  private int frontRight_CAN = 3;
  private int backRight_CAN = 4;
  private int leftShooter_CAN = 5;
  private int rightShooter_CAN = 6;
  private int frontIndexer_CAN = 7;
  private int middleIndexer_CAN = 8;
  private int backIndexer_CAN = 9;
  private int pickUp_CAN = 10;
  private int frontFlight_CAN = 11;
  private int midFlight_CAN = 12;
  private int backFlight_CAN = 13;

  // xbox thing
  private XboxController xbox;

  // drivetrain thing
  private WPI_TalonFX frontLeft;
  private WPI_TalonFX backLeft;
  private WPI_TalonFX frontRight;
  private WPI_TalonFX backRight;
  private SpeedControllerGroup leftSide;
  private SpeedControllerGroup rightSide;
  private DifferentialDrive driveTrain;
  private double rampInSec = 0.1875;

  // gyro thing
  private AHRS navx;
  private double increment = 0.2;
  private double desiredAngle = 0.0;
  private boolean compensating = false;

  //shooter things
  private WPI_TalonFX shooterLeft;
  private WPI_TalonFX shooterRight;
  private double howChange = .01;
  private double currentSpeed = -.29;

  // indexer things
  private WPI_TalonFX frontIndexer;
  private WPI_TalonFX middleIndexer;
  private WPI_TalonFX backIndexer;
  private TimeOfFlight frontFlight;
  private TimeOfFlight midFlight;
  private TimeOfFlight backFlight;
  private double indexerSpeed = .30;
  private int numOfCells = 0;
  private double indexerThresh = 3.0;

  // tof sensor things

  // intake things
  private WPI_TalonFX pickUp;
  private double pickUpSpeed = -1.0;

  // state variables
  private final int DRIVING = 0;
  private final int SHOOTING = 1;
  private final int INDEXING = 2;
  private final int PICKUPPING = 3;
  private final int SHOOTING_INDEXING = 4;
  private final int DRIVING_PICKUPPING = 5;
  private final int GYRO_DRIVING = 6;
  private final int INDEXING_USING_SENSORS = 7;
  private final int DRIVING_SHOOTING_INDEXING = 8;

  private int currentState = DRIVING_SHOOTING_INDEXING;

  public void initDriveTrain() {
    frontRight = new WPI_TalonFX(frontRight_CAN);
    backRight = new WPI_TalonFX(backRight_CAN);
    frontLeft = new WPI_TalonFX(frontLeft_CAN);
    backLeft = new WPI_TalonFX(backLeft_CAN);
    frontRight.configOpenloopRamp(rampInSec);
    backRight.configOpenloopRamp(rampInSec);
    frontLeft.configOpenloopRamp(rampInSec);
    backLeft.configOpenloopRamp(rampInSec);
    leftSide = new SpeedControllerGroup(frontLeft, backLeft);
    rightSide = new SpeedControllerGroup(frontRight, backRight);
    driveTrain = new DifferentialDrive(leftSide, rightSide);
  }

  public void setFlightMode(RangingMode mode, int rate)
  {
    if(frontFlight != null && midFlight != null && backFlight != null)
    {
      frontFlight.setRangingMode(mode, rate);
      midFlight.setRangingMode(mode, rate);
      backFlight.setRangingMode(mode, rate);
    }
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    // m_chooser.addOption("My Auto", kCustomAuto);
    // SmartDashboard.putData("Auto choices", m_chooser);

    if ( currentState == SHOOTING )
    {
      shooterLeft = new WPI_TalonFX(leftShooter_CAN);
      shooterRight = new WPI_TalonFX(rightShooter_CAN);
    }
    else if ( currentState == INDEXING )
    {
      frontIndexer = new WPI_TalonFX(frontIndexer_CAN);
      middleIndexer = new WPI_TalonFX(middleIndexer_CAN);
      backIndexer = new WPI_TalonFX(backIndexer_CAN);
    }
    else if ( currentState == PICKUPPING )
    {
      pickUp = new WPI_TalonFX(pickUp_CAN);
    }
    else if ( currentState == SHOOTING_INDEXING )
    {
      shooterLeft = new WPI_TalonFX(leftShooter_CAN);
      shooterRight = new WPI_TalonFX(rightShooter_CAN);
      frontIndexer = new WPI_TalonFX(frontIndexer_CAN);
      middleIndexer = new WPI_TalonFX(middleIndexer_CAN);
      backIndexer = new WPI_TalonFX(backIndexer_CAN); 
    }
    else if (currentState == DRIVING)
    {
      initDriveTrain();
      dontBreakMe(NeutralMode.Brake);
    }
    else if (currentState == DRIVING_PICKUPPING)
    {
      initDriveTrain();
      dontBreakMe(NeutralMode.Brake);
      pickUp = new WPI_TalonFX(pickUp_CAN);
    }
    else if (currentState == GYRO_DRIVING)
    {
      initDriveTrain();
      dontBreakMe(NeutralMode.Brake);
      navx = new AHRS(SPI.Port.kMXP);
    }
    else if (currentState == INDEXING_USING_SENSORS)
    {
      frontIndexer = new WPI_TalonFX(frontIndexer_CAN);
      middleIndexer = new WPI_TalonFX(middleIndexer_CAN);
      backIndexer = new WPI_TalonFX(backIndexer_CAN);
      frontFlight = new TimeOfFlight(frontFlight_CAN);
      midFlight = new TimeOfFlight(midFlight_CAN);
      backFlight = new TimeOfFlight(backFlight_CAN);
      setFlightMode(RangingMode.Long, 24);
    }
    else if (currentState == DRIVING_SHOOTING_INDEXING)
    {
      frontIndexer = new WPI_TalonFX(frontIndexer_CAN);
      middleIndexer = new WPI_TalonFX(middleIndexer_CAN);
      backIndexer = new WPI_TalonFX(backIndexer_CAN);
      shooterLeft = new WPI_TalonFX(leftShooter_CAN);
      shooterRight = new WPI_TalonFX(rightShooter_CAN);
      // PICKUP
      pickUp = new WPI_TalonFX(pickUp_CAN);
      initDriveTrain();
      dontBreakMe(NeutralMode.Brake);
      navx = new AHRS(SPI.Port.kMXP);
    }

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
  public void autonomousInit() {
    // m_autoSelected = m_chooser.getSelected();
    // // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    // System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // switch (m_autoSelected) {
    //   case kCustomAuto:
    //     // Put custom auto code here
    //     break;
    //   case kDefaultAuto:
    //   default:
    //     // Put default auto code here
    //     break;
    // }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    if(navx != null) {
      navx.reset();
    }
  }

  public void dontBreakMe(NeutralMode mode) {
    frontLeft.setNeutralMode(mode);
    frontRight.setNeutralMode(mode);
    backLeft.setNeutralMode(mode);
    backRight.setNeutralMode(mode);
  }

  public void shoot() 
  {
    if (xbox.getYButton() == true && currentSpeed <= 1.0) {
      currentSpeed = currentSpeed + howChange;
      System.out.println("WARNING! currSpeed: " + currentSpeed);
      //shooterRight.set(currentSpeed);
    }

    if (xbox.getAButton() == true && currentSpeed >= -1.0) {
      currentSpeed = currentSpeed - howChange;
      System.out.println("WARNING! currSpeed: " + currentSpeed);
      //shooterRight.set(currentSpeed);
    }

    if(xbox.getXButton()) {
      System.out.println("WARNING! currSped: " + currentSpeed);
    }

    if (xbox.getBButton() == true) {
      shooterLeft.set(-currentSpeed);
      shooterRight.set(currentSpeed);
      getSpeeds();
      //shooterRight.set(neutralSpeed);
    } else {
      shooterLeft.set(0.0);
      shooterRight.set(0.0);
    }
  }

  public void index()
  {
    if (xbox.getBButton() == true) 
    {
      frontIndexer.set(-indexerSpeed);
      middleIndexer.set(indexerSpeed);
      backIndexer.set(indexerSpeed);
    }
    else
    {
      frontIndexer.set(0);
      middleIndexer.set(0);
      backIndexer.set(0);
    }
  }

  public void indexUsingSensors() {
    numOfCells = getBallCount();
    switch(numOfCells) {
      case 0:
        indexUntilLoaded(frontIndexer, frontFlight, indexerThresh);
        break;
      case 1:
        // indexUntilLoaded(frontIndexer)
        break;
      case 2:
        // do other other stuff
        break;
      case 3:
        // do other other other stuff
        break;
    }
  }

  public int getBallCount() {
    int count = 0;
    if(frontFlight.getRange() / 25.4 <= indexerThresh)
    {
      count++;
    }
    if(midFlight.getRange() / 25.4 <= indexerThresh)
    {
      count++;
    }
    if(backFlight.getRange() / 25.4 <= indexerThresh)
    {
      count++;
    }
    return count;
  }

  public void indexUntilLoaded(WPI_TalonFX motor, TimeOfFlight sensor, double thresh)
  {

  }

  public void pickUp()
  {
    if (xbox.getBumper(Hand.kRight) == true) 
    {
      pickUp.set(pickUpSpeed);
    }
    else
    {
      pickUp.set(0);
    }
  }

  public void drive()
  {
    if(!compensating) {
      // leftPower = (frontLeft.getMotorOutputPercent() + backLeft.getMotorOutputPercent()) / 2.0;
      // rightPower = (frontRight.getMotorOutputPercent() + backRight.getMotorOutputPercent()) / 2.0;
      driveTrain.arcadeDrive(-xbox.getRawAxis(1), xbox.getRawAxis(4)); 
    } else {
      if(navx.getAngle() <= desiredAngle) {
        driveTrain.arcadeDrive(-xbox.getRawAxis(1), xbox.getRawAxis(4) + increment);
      }
      else if(navx.getAngle() >= desiredAngle) {
        driveTrain.arcadeDrive(-xbox.getRawAxis(1), xbox.getRawAxis(4) - increment);
      }
    }
  }

  public void gyroCompensation() {
    if(xbox.getBumper(Hand.kLeft)) {
      compensating = true;
    } else {
      desiredAngle = navx.getAngle();
      compensating = false;
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if ( currentState == SHOOTING )
      shoot();
    else if ( currentState == INDEXING )
      index();
    else if ( currentState == PICKUPPING )
      pickUp();
    else if ( currentState == SHOOTING_INDEXING )
    {
      shoot();
      index();
    }
    else if ( currentState == DRIVING_PICKUPPING )
    {
      drive();
      pickUp();
    }
    else if ( currentState == GYRO_DRIVING )
    {
      drive();
      gyroCompensation();
    }
    else if (currentState == DRIVING_SHOOTING_INDEXING)
    {
      drive();
      gyroCompensation();
      shoot();
      index();
      pickUp();
    }
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
  public void testPeriodic() {}

  public double getLeftEncoderVelocity() { return shooterLeft.getSelectedSensorVelocity(); }
  public double getRightEncoderVelocity() { return shooterRight.getSelectedSensorVelocity(); }

  public void getSpeeds() 
  {
    // returns wheel speeds in meters per second
    //double leftVelocity = Units.inchesToMeters( getLeftEncoderVelocity() / PULSE_PER_INCH );
    //double rightVelocity = Units.inchesToMeters( getLeftEncoderVelocity() / PULSE_PER_INCH );
    double leftRotationsPerSecond = (double) getLeftEncoderVelocity() / 2048 / .5 * 10;        
    double leftVelocity = leftRotationsPerSecond * 2 * Math.PI * Units.inchesToMeters( 2 );
    double rightRotationsPerSecond = (double) getRightEncoderVelocity() / 2048 / .5 * 10;        
    double rightVelocity = rightRotationsPerSecond * 2 * Math.PI * Units.inchesToMeters( 2 );

    System.out.println("WARNING!" + " Left side velocity: " + leftVelocity);
    System.out.println("WARNING! Ride side velocity: " + rightVelocity);
  }
}
