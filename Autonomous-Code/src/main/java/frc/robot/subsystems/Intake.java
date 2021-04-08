// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private WPI_TalonFX intakeMotor;

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new WPI_TalonFX(kIntakeID);
  }

  public void pickUp() {
    intakeMotor.set(kIntakeSpeed);
  }

  public void stop() {
    intakeMotor.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
