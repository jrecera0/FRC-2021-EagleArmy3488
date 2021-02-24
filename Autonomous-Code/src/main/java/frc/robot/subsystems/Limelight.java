// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private double tx;
  private double ty;
  private double ta;
  private NetworkTable networkTable;
  
  /** Creates a new Limelight. */
  public Limelight() {
    networkTable = NetworkTableInstance.getDefault().getTable("limelight");
  }

  @Override
  public void periodic() {
    tx = networkTable.getEntry("tx").getDouble(0.0);
    ty = networkTable.getEntry("ty").getDouble(0.0);
    ta = networkTable.getEntry("ta").getDouble(0.0);
  }

  public double getTX() {
    return tx;
  }

  public double getTY() {
    return ty;
  }

  public double getTA() {
    return ta;
  }
}
