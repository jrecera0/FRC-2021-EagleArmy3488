// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private Drivetrain driveTrainVar;
  public RamseteCommand rCommand;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    if(driveTrainVar == null) driveTrainVar = new Drivetrain();

    configureButtonBindings();
  }

  public void resetGyro()
    {
        driveTrainVar.resetGyro();
    }

  private Trajectory createTrajectory()
  {
    TrajectoryConfig config = new TrajectoryConfig( Units.feetToMeters( 6 ), Units.feetToMeters( 4 ) );
    config.setKinematics( driveTrainVar.getKinematics() );

    Trajectory tempTrajectory = TrajectoryGenerator.generateTrajectory(
        Arrays.asList( new Pose2d(), new Pose2d( Units.feetToMeters(30), 0, new Rotation2d(0) ) ),

        config
    ); // WRONG TRAJECTORY; CURRENTLY USED
    
    Trajectory forwardTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
        List.of(), new Pose2d(Units.feetToMeters(15), 0, new Rotation2d()),
        config
    );

    Trajectory alternateTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(Units.feetToMeters(5), 0), // 5 Feet forward
            new Translation2d(Units.feetToMeters(5), Units.feetToMeters(5)), // Turn left and move 5 feet
            new Translation2d(Units.feetToMeters(10), Units.feetToMeters(5))
        ),
        new Pose2d(0, 0, new Rotation2d()), config);
    
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(
        new Translation2d(Units.feetToMeters(5), Units.feetToMeters(2.5)),
        new Translation2d(Units.feetToMeters(10), Units.feetToMeters(5)),
        new Translation2d(Units.feetToMeters(12.5), Units.feetToMeters(-2.5)),
        new Translation2d(Units.feetToMeters(12.5), Units.feetToMeters(7.5)),
        new Translation2d(Units.feetToMeters(15), Units.feetToMeters(0)),
        new Translation2d(Units.feetToMeters(20), Units.feetToMeters(2.5))
    ),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(Units.feetToMeters(30), Units.feetToMeters(0), new Rotation2d(0)),
    // Pass config
    config
    );
    //return tempTrajectory;
    return trajectory;
  }

  public Command getAutonomousCommand()
    {
        driveTrainVar.resetOdometry( new Pose2d() );

        Trajectory trajectory = createTrajectory();

        RamseteCommand command = new RamseteCommand(
            trajectory,
            driveTrainVar::getPose,
            new RamseteController( Drivetrain.kRamseteB, Drivetrain.kRamseteZeta ),
            driveTrainVar.getSimpleMotorFeedForward(),
            driveTrainVar.getKinematics(),
            driveTrainVar::getSpeeds,
            driveTrainVar.getLeftPIDCOntroller(),
            driveTrainVar.getRightPIDCOntroller(),
            driveTrainVar::tankDriveVolts,
            driveTrainVar
        );

        if ( rCommand == null ) rCommand = command;
        return command;
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
}
