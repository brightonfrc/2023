// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class WaitForArmToSettle extends CommandBase {
  private Arm arm;
  private double acceptableError;

  /** Creates a new WaitForArmToSettle. */
  public WaitForArmToSettle(Arm arm, double acceptableError) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.acceptableError = acceptableError;

    addRequirements(arm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double error = arm.chainMotorEncoder.getPosition() - arm.chainMotorDesiredPosition;
    return Math.abs(error) < acceptableError;
  }
}
