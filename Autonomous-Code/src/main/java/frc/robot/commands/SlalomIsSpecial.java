// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Trajectories;

public class SlalomIsSpecial extends CommandBase {
  // Normal stuff
  private DriveTrain driveTrain;
  private Trajectories paths;

  // Added more commands to make this even more jank
  private RamseteCommand commandSect1;
  private RamseteCommand commandSect2;
  private RamseteCommand commandSect3;
  private RamseteCommand commandSect4;
  private Trajectory trajectory1;
  private Trajectory trajectory2;
  private Trajectory trajectory3;
  private Trajectory trajectory4;

  // needed a counter
  private int section;

  /** Creates a new JsonTrajectory. */
  public SlalomIsSpecial(DriveTrain drive, Trajectories trajectories) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = drive;
    paths = trajectories;
    section = 1;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Make sure odometry is reset
    driveTrain.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0)));

    // Attempt to open the the proper trajectory
    trajectory1 = paths.getSlalomPathSect(1);
    trajectory2 = paths.getSlalomPathSect(2);
    trajectory3 = paths.getSlalomPathSect(3);
    trajectory4 = paths.getSlalomPathSect(4);

    // If successful, follow the trajectory.
    if(trajectory1 != null) {
      // Move robot trajectory to right pos
      Transform2d transform = driveTrain.getPose().minus(trajectory1.getInitialPose());
      trajectory1 = trajectory1.transformBy(transform);

      // Run the Ramsete Command to make the robot drive around
      commandSect1 = new RamseteCommand(
        trajectory1,
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
    }
    commandSect1.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(section == 1 && commandSect1.isFinished()) {
      if(trajectory2 != null) {
        // Move robot trajectory to right pos
        driveTrain.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0)));
        Transform2d transform = driveTrain.getPose().minus(trajectory2.getInitialPose());
        trajectory2 = trajectory2.transformBy(transform);
  
        // Run the Ramsete Command to make the robot drive around
        commandSect2 = new RamseteCommand(
          trajectory2,
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
      }
      //driveTrain.invertDriveTrain();
      commandSect2.schedule();
      section++;
    }
    if(section == 2 && commandSect2.isFinished()) {
      if(trajectory3 != null) {
        // Move robot trajectory to right pos
        driveTrain.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0)));
        Transform2d transform = driveTrain.getPose().minus(trajectory3.getInitialPose());
        trajectory3 = trajectory3.transformBy(transform);
  
        // Run the Ramsete Command to make the robot drive around
        commandSect3 = new RamseteCommand(
          trajectory3,
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
      }
      //driveTrain.invertDriveTrain();
      commandSect3.schedule();
      section++;
    }
    if(section == 3 && commandSect3.isFinished()) {
      if(trajectory4 != null) {
        // Move robot trajectory to right pos
        driveTrain.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0)));
        Transform2d transform = driveTrain.getPose().minus(trajectory4.getInitialPose());
        trajectory4 = trajectory4.transformBy(transform);
  
        // Run the Ramsete Command to make the robot drive around
        commandSect4 = new RamseteCommand(
          trajectory4,
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
      }
      //driveTrain.invertDriveTrain();
      commandSect4.schedule();
      section++;
    }
    if(section == 4 && commandSect4.isFinished()) {
      //driveTrain.invertDriveTrain();
      section = 0;
      System.out.println("WARNING! Completed Slalom.");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
