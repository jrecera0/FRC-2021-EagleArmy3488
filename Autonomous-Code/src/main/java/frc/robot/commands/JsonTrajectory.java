// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.TrajectoryPathnames.*;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.DriveTrain;

public class JsonTrajectory extends CommandBase {
  private DriveTrain driveTrain;
  private Blinkin ledController;
  private Trajectory trajectory;
  private RamseteCommand command;
  //private TrajectoryConfig config;
  private String trajectoryPathname;

  /** Creates a new JsonTrajectory. */
  public JsonTrajectory(DriveTrain drive, Blinkin leds) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = drive;
    ledController = leds;
    trajectoryPathname = kLinePath; // CHANGE HERE TO CHANGE WHAT PATH IS RUN
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Make sure odometry is reset
    driveTrain.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0)));

    // Attempt to open the filepath for the trajectory
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryPathname);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryPathname, ex.getStackTrace());
    }
    // TrajectoryConfig config = new TrajectoryConfig(kVelocityMax, kAccelerationMax);
    // trajectory = TrajectoryGenerator.generateTrajectory(
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     List.of(
    //     ),
    //     new Pose2d(Units.feetToMeters(10), Units.feetToMeters(0), new Rotation2d(0)),
    //     // Pass config
    //     config
    //   );

    // If successful, follow the trajectory.
    if(trajectory != null) {
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
      ledController.setWhite();
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
    return command.isFinished();
  }
}
