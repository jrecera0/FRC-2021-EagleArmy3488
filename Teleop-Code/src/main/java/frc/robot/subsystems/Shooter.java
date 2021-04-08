// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private WPI_TalonFX leftShooter;
  private WPI_TalonFX rightShooter;
  private double speed;

  /** Creates a new Shooter. */
  public Shooter() {
    leftShooter = new WPI_TalonFX(kLeftShooterID);
    rightShooter = new WPI_TalonFX(kRightShooterID);
    speed = kDefaultSpeed;
  }

  public void shoot() {
    leftShooter.set(-speed);
    rightShooter.set(speed);
  }

  public void setSpeedManual(double speed) {
    leftShooter.set(-speed);
    rightShooter.set(speed);
  }

  public void setGreenZone() {
    speed = kGreenZoneSpeed;
    System.out.println("WARNING! Green zone set.");
  }

  public void setYellowZone() {
    speed = kYellowZoneSpeed;
    System.out.println("WARNING! Yellow zone set.");
  }

  public void setBlueZone() {
    speed = kBlueZoneSpeed;
    System.out.println("WARNING! Blue zone set.");
  }

  public void setRedZone() {
    speed = kRedZoneSpeed;
    System.out.println("WARNING! Red zone set.");
  }

  public void stop() {
    leftShooter.set(0.0);
    rightShooter.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
