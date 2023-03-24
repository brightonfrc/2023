// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.dataStorageClasses;

/**
 * A class to store counts of encoders in different arm positions
 */
public class TurntablePositionCounts {
  public double encoderCounts;
  public String name;
  
  public TurntablePositionCounts (double encoderCounts, String name) {
    this.encoderCounts = encoderCounts;
    this.name = name;
  }
}
