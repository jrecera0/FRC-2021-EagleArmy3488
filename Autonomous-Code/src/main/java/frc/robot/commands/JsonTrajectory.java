// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.TrajectoryPathnames.*;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Trajectories;

public class JsonTrajectory extends CommandBase {
  private DriveTrain driveTrain;
  private Trajectory trajectory;
  private RamseteCommand command;
  private SlalomIsSpecial slalomCommand;
  private Trajectories paths;
  private String currPath;

  /** Creates a new JsonTrajectory. */
  public JsonTrajectory(DriveTrain drive, Trajectories trajectories) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = drive;
    currPath = kCurrentTraj;
    paths = trajectories;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Make sure odometry is reset
    driveTrain.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0)));

    // Attempt to set proper trajectory
    if(currPath.equals("Bounce")) {
      trajectory = paths.getBouncePath();
    }
    if(currPath.equals("Barrel")) {
      trajectory = paths.getBarrelRacingPath();
    }
    if(currPath.equals("Test")) {
      trajectory = paths.getTestPath();
    }
    if(currPath.equals("Slalom")) {
      trajectory = paths.getSlalomPath();
    }
    if(currPath.equals("SlalomIsSpecial")) {
      slalomCommand = new SlalomIsSpecial(driveTrain, paths);
      slalomCommand.schedule();
    }

    // If successful, follow the trajectory.
    if(trajectory != null && slalomCommand == null) {
      // ADDED 3/23/21
      // Translate Trajectory to Robot Pos
      Transform2d transform = driveTrain.getPose().minus(trajectory.getInitialPose());
      trajectory = trajectory.transformBy(transform);

      // Run the Ramsete Command to make the robot drive around
      command = new RamseteCommand(
        trajectory,
        driveTrain::getPose,
        new RamseteController(kRamseteB, kRamseteZeta),
        driveTrain.getSimpleMotorFeedForward(),
        driveTrain.getKinematics(),
        driveTrain::getDriveWheelSpeeds,
        driveTrain.getLeftPIDController(),
        driveTrain.getRightPIDController(),
        driveTrain::setTankDriveVolts,
        driveTrain
      );
      command.schedule();
    }
    else {
      System.out.println("WARNING! No path found.");
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false;
    if(slalomCommand == null) {
      return command.isFinished();
    } else {
      return slalomCommand.isFinished();
    }
  }
}
