// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DifferentialDriveWrapper;

public final class Autos {
  /** Runs Path Planner Path Following */
  public static CommandBase pathPlannerAuto(DifferentialDriveWrapper drive) {
    return new AutoPathPlanner(drive);
  }
  
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
