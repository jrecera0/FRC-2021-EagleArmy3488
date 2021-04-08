// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.FieldPositioning.*;
import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Trajectories;

public class DeterminePath extends CommandBase {
  /** Creates a new DeterminePath. */
  private Limelight limelight;
  private DriveTrain driveTrain;
  private Trajectory trajectory;
  private Trajectory initTraj;
  private Trajectories paths;
  private RamseteCommand command;
  private RamseteCommand initCommand;
  private Intake intake;
  private boolean isInitDone;

  public DeterminePath(DriveTrain drive, Intake pickUp, Limelight fieldVision, Trajectories trajectories) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = drive;
    intake = pickUp;
    limelight = fieldVision;
    paths = trajectories;
    isInitDone = false;
    addRequirements(driveTrain, limelight);
  }

  // Called when the command is initially scheduled.
  // Plan here is to figure out what path we're taking
  @Override
  public void initialize() {
    // Determing the correct path
    if(isPathARed() == true) {
      System.out.println("WARNING! Picked A_RED");
      trajectory = paths.getPathARed();
    }
    else if(isPathBRed() == true) {
      System.out.println("WARNING! Picked B_RED");
      trajectory = paths.getPathBRed();
    }
    else if(isPathABlue() == true) {
      System.out.println("WARNING! Picked A_BLUE");
      trajectory = paths.getPathABlue();
    }
    else if(isPathBBlue() == true) {
      System.out.println("WARNING! Picked B_BLUE");
      trajectory = paths.getPathBBlue();
    }
    
    initTraj = paths.getTestPath();

    // If path found, run the path
    if(trajectory != null) {
      // Make sure odometry is reset
      driveTrain.resetOdometry(new Pose2d());

      initCommand = new RamseteCommand(
        initTraj,
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
      initCommand.schedule();
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
    } else {
      System.out.println("WARNING! No path found.");
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  // TODO: Add polling system to determine path?
  @Override
  public void execute() {
    if(initCommand.isFinished() && !isInitDone) {
      // Make sure odometry is reset
      driveTrain.resetOdometry(new Pose2d());

      // Run the Ramsete Command to make the robot drive around
      command.schedule();
      isInitDone = true;
    }
    intake.pickUp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return command.isFinished();
  }
  
  // Limelight checks
  private boolean isWithinThresh(double actual, double obtained, double thresh) {
    if (obtained - thresh > actual || obtained + thresh < actual)
      return false;
    else
      return true;
  }

  private boolean isPathARed() {
    boolean goForPath = false;
    double current_tx = limelight.getTX();
    double current_ty = limelight.getTY();

    boolean correct_tx = isWithinThresh(aRed_tx, current_tx, kInterval);
    boolean correct_ty = isWithinThresh(aRed_ty, current_ty, kInterval);
    
    if (correct_tx == true && correct_ty == true)
      goForPath = true;

    return goForPath;
  }

  private boolean isPathBRed() {
    boolean goForPath = false;
    double current_tx = limelight.getTX();
    double current_ty = limelight.getTY();

    boolean correct_tx = isWithinThresh(bRed_tx, current_tx, kInterval);
    boolean correct_ty = isWithinThresh(bRed_ty, current_ty, kInterval);
    
    if (correct_tx == true && correct_ty == true)
      goForPath = true;

    return goForPath;
  }

  private boolean isPathABlue() {
    boolean goForPath = false;
    double current_tx = limelight.getTX();
    double current_ty = limelight.getTY();

    boolean correct_tx = isWithinThresh(aBlue_tx, current_tx, kInterval);
    boolean correct_ty = isWithinThresh(aBlue_ty, current_ty, kInterval);
    
    if (correct_tx == true && correct_ty == true)
      goForPath = true;

    return goForPath;
  }

  private boolean isPathBBlue() {
    boolean goForPath = false;
    double current_tx = limelight.getTX();
    double current_ty = limelight.getTY();

    boolean correct_tx = isWithinThresh(bBlue_tx, current_tx, kInterval);
    boolean correct_ty = isWithinThresh(bBlue_ty, current_ty, kInterval);
    
    if (correct_tx == true && correct_ty == true)
      goForPath = true;

    return goForPath;
  }
}
