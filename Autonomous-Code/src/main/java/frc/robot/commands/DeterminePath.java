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
import frc.robot.subsystems.Limelight;

public class DeterminePath extends CommandBase {
  /** Creates a new DeterminePath. */
  private Limelight limelight;
  private DriveTrain driveTrain;
  private Trajectory trajectory;
  private int tx;
  private int ty;
  private int ta;

  public DeterminePath(DriveTrain drive, Limelight fieldVision) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = drive;
    limelight = fieldVision;
    addRequirements(driveTrain, limelight);
  }

  // Called when the command is initially scheduled.
  // Plan here is to figure out what path we're taking
  @Override
  public void initialize() {
    if(isPathARed()) {
      trajectory = new Trajectory();
    }
    driveTrain.resetOdometry(new Pose2d());

    RamseteCommand command = new RamseteCommand(
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

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
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

    boolean correct_tx = isWithinThresh(aRed_tx, current_tx, .5);
    boolean correct_ty = isWithinThresh(aRed_ty, current_ty, .5);
    
    if (correct_tx == true && correct_ty == true)
      goForPath = true;

    return goForPath;
  }
}
