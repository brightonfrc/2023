// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.dataStorageClasses;

/**
 * A class to store counts of encoders in different arm positions
 */
public class ArmPositionCounts {
  public double chainMotorCounts;
  public double cableMotorCounts;
  public String name;
  
  public ArmPositionCounts (double chainMotorCounts, double cableMotorCounts, String name) {
    this.chainMotorCounts = chainMotorCounts;
    this.cableMotorCounts = cableMotorCounts;
    this.name = name;
  }
}
