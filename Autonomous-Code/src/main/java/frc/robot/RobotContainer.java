// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Challenge.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DeterminePath;
import frc.robot.commands.JsonTrajectory;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Trajectories;

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
  private JsonTrajectory jsonTrajectory;
  private Trajectories trajectories;
  private Intake pickUp;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Subsystems
    driveTrain = new DriveTrain();
    limelight = new Limelight();
    pickUp = new Intake();

    // Needed for trajectory generation
    trajectories = new Trajectories(driveTrain);

    // Commands
    determinePath = new DeterminePath(driveTrain, pickUp, limelight, trajectories);
    jsonTrajectory = new JsonTrajectory(driveTrain, trajectories);
    
    // Lol we don't use this
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
    if(kCurrChallenge.equals("AutoNav")) {
      return jsonTrajectory;
    }
    if(kCurrChallenge.equals("GSearch")) {
      return determinePath;
    }
    return null; // Default if something invalid is passed, will crash the robot code.
  }
}
