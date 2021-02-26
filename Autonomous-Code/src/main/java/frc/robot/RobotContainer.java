// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.DeterminePath;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer { 
  // The robot's subsystems and commands are defined here...
  private DriveTrain driveTrain;
  private Limelight limelight;
  private DeterminePath determinePath;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveTrain = new DriveTrain();
    limelight = new Limelight();
    determinePath = new DeterminePath(driveTrain, limelight);
    configureButtonBindings(); // Configure the button bindings
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return determinePath;
  }
  private Trajectory createARedTrajectory()
  {
    new Pose2d(0, 0, new Rotation2d(0));

    List.of(
      new Translation2d(Units.feetToMeters(5), Units.feetToMeters(-2.5));
      new Translation2d(Units.feetToMeters(10), Units.feetToMeters(-5));
      new Translation2d(Units.feetToMeters(12.5), Units.feetToMeters(-7.5));
      new Translation2d(Units.feetToMeters(25), Units.feetToMeters(0));
    );
    new Pose2d(Units.feetToMeters(30), Units.feetToMeters(0), new Rotation2d(0));
    // Pass config
    config
    );
    Trajectory trajectory;
    
    //return tempTrajectory;
    return trajectory;
    //return alternateTrajectory;


  }
  private Trajectory createBRedTrajectory()
  {
    new Pose2d(0, 0, new Rotation2d(0));

    List.of(
      new Translation2d(Units.feetToMeters(5), Units.feetToMeters(0));
      new Translation2d(Units.feetToMeters(10), Units.feetToMeters(-5));
      new Translation2d(Units.feetToMeters(15), Units.feetToMeters(0));
      new Translation2d(Units.feetToMeters(25), Units.feetToMeters(0));
    );
    new Pose2d(Units.feetToMeters(30), Units.feetToMeters(0), new Rotation2d(0));
    // Pass config
    config
    );
    Trajectory trajectory;
    
    //return tempTrajectory;
    return trajectory;
    //return alternateTrajectory;


  }
  private Trajectory createABlueTrajectory()
  {
    new Pose2d(0, 0, new Rotation2d(0));

    List.of(
      new Translation2d(Units.feetToMeters(12.5), Units.feetToMeters(2.5));
      new Translation2d(Units.feetToMeters(15), Units.feetToMeters(0));
      new Translation2d(Units.feetToMeters(20), Units.feetToMeters(-2.5));
      new Translation2d(Units.feetToMeters(25), Units.feetToMeters(0));
    );
    new Pose2d(Units.feetToMeters(30), Units.feetToMeters(0), new Rotation2d(0));
    // Pass config
    config
    );
    Trajectory trajectory;
    
    //return tempTrajectory;
    return trajectory;
    //return alternateTrajectory;


  }
  private Trajectory createBBlueTrajectory()
  {
    new Pose2d(0, 0, new Rotation2d(0));

    List.of(
      new Translation2d(Units.feetToMeters(12.5), Units.feetToMeters(0));
      new Translation2d(Units.feetToMeters(17.5), Units.feetToMeters(5));
      new Translation2d(Units.feetToMeters(22.5), Units.feetToMeters(0));
      new Translation2d(Units.feetToMeters(25), Units.feetToMeters(0));
    );
    new Pose2d(Units.feetToMeters(30), Units.feetToMeters(0), new Rotation2d(0));
    // Pass config
    config
    );
    Trajectory trajectory;
    
    //return tempTrajectory;
    return trajectory;
    //return alternateTrajectory;


  }








}
