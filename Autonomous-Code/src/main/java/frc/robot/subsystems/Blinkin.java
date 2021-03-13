// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.BlinkinConstants.*;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Blinkin extends SubsystemBase {
  /** Creates a new Blinkin. */
  private Spark ledController;

  public Blinkin() {
    ledController = new Spark(kBlinkinPort);
  }

  public void setRed() {
    ledController.set(kRed);
  }

  public void setOrange() {
    ledController.set(kOrange);
  }

  public void setYellow() {
    ledController.set(kYellow);
  }

  public void setGreen() {
    ledController.set(kGreen);
  }

  public void setBlue() {
    ledController.set(kBlue);
  }

  public void setViolet() {
    ledController.set(kViolet);
  }

  public void setBlack() {
    ledController.set(kBlack);
  }

  public void setWhite() {
    ledController.set(kWhite);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
