// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.FieldPositioning.*;

import java.util.List;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class DeterminePath extends CommandBase {
  /** Creates a new DeterminePath. */
  private Limelight limelight;
  private DriveTrain driveTrain;
  private Trajectory trajectory;
  private RamseteCommand command;
  private int tx;
  private int ty;
  private int ta;
  private TrajectoryConfig config; 

  public DeterminePath(DriveTrain drive, Limelight fieldVision) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = drive;
    limelight = fieldVision;
    config = new TrajectoryConfig(Units.feetToMeters(6), Units.feetToMeters(4));
    config.setKinematics(driveTrain.getKinematics());
    addRequirements(driveTrain, limelight);
  }

  // Called when the command is initially scheduled.
  // Plan here is to figure out what path we're taking
  @Override
  public void initialize() {
    System.out.println("WARNING! TEST");
    // NEED TO UPDATE FOR REAL LOGIC AND REAL TRAJECTORIES
    if(isPathARed() == true) {
      trajectory =  TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),

        List.of(
          new Translation2d(Units.feetToMeters(5), Units.feetToMeters(-2.5)),
          new Translation2d(Units.feetToMeters(10), Units.feetToMeters(-5)),
          new Translation2d(Units.feetToMeters(12.5), Units.feetToMeters(-7.5)),
          new Translation2d(Units.feetToMeters(25), Units.feetToMeters(0))
        ),
        new Pose2d(Units.feetToMeters(30), Units.feetToMeters(0), new Rotation2d(0)),
        // Pass config
        config
      );
    }
    if(isPathBRed() == true ) {
      trajectory =  TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),

        List.of(
          new Translation2d(Units.feetToMeters(5), Units.feetToMeters(0)),
          new Translation2d(Units.feetToMeters(10), Units.feetToMeters(-5)),
          new Translation2d(Units.feetToMeters(15), Units.feetToMeters(0)),
          new Translation2d(Units.feetToMeters(25), Units.feetToMeters(0))
        ),
        new Pose2d(Units.feetToMeters(30), Units.feetToMeters(0), new Rotation2d(0)),
        // Pass config
        config
      );
    }
    if(isPathABlue() == true ) {
      trajectory =  TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),

        List.of(
          new Translation2d(Units.feetToMeters(12.5), Units.feetToMeters(2.5)),
          new Translation2d(Units.feetToMeters(15), Units.feetToMeters(0)),
          new Translation2d(Units.feetToMeters(20), Units.feetToMeters(-2.5)),
          new Translation2d(Units.feetToMeters(25), Units.feetToMeters(0))
        ),
        new Pose2d(Units.feetToMeters(30), Units.feetToMeters(0), new Rotation2d(0)),
        // Pass config
        config
      );
    }
    if(isPathBBlue() == true ) {
      trajectory =  TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),

        List.of(
          new Translation2d(Units.feetToMeters(12.5), Units.feetToMeters(0)),
          new Translation2d(Units.feetToMeters(17.5), Units.feetToMeters(5)),
          new Translation2d(Units.feetToMeters(22.5), Units.feetToMeters(0)),
          new Translation2d(Units.feetToMeters(25), Units.feetToMeters(0))
        ),
        new Pose2d(Units.feetToMeters(30), Units.feetToMeters(0), new Rotation2d(0)),
        // Pass config
        config
      );
    }

    trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(),
      new Pose2d(0, 1, new Rotation2d(0)),
      config
    );

    driveTrain.resetOdometry(new Pose2d());

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

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return command.isFinished();
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
  private boolean isPathBRed() {
    boolean goForPath = false;
    double current_tx = limelight.getTX();
    double current_ty = limelight.getTY();

    boolean correct_tx = isWithinThresh(bRed_tx, current_tx, .5);
    boolean correct_ty = isWithinThresh(bRed_ty, current_ty, .5);
    
    if (correct_tx == true && correct_ty == true)
      goForPath = true;

    return goForPath;
  }
  private boolean isPathABlue() {
    boolean goForPath = false;
    double current_tx = limelight.getTX();
    double current_ty = limelight.getTY();

    boolean correct_tx = isWithinThresh(aBlue_tx, current_tx, .5);
    boolean correct_ty = isWithinThresh(aBlue_ty, current_ty, .5);
    
    if (correct_tx == true && correct_ty == true)
      goForPath = true;

    return goForPath;
  }
  private boolean isPathBBlue() {
    boolean goForPath = false;
    double current_tx = limelight.getTX();
    double current_ty = limelight.getTY();

    boolean correct_tx = isWithinThresh(bBlue_tx, current_tx, .5);
    boolean correct_ty = isWithinThresh(bBlue_ty, current_ty, .5);
    
    if (correct_tx == true && correct_ty == true)
      goForPath = true;

    return goForPath;
  }
}
