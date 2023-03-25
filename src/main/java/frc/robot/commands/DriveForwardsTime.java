// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DifferentialDriveWrapper;

public class DriveForwardsTime extends CommandBase {
  DifferentialDriveWrapper m_drivetrain;
  long duration;
  double speed;
  long startTime;
  /** Creates a new DriveForwards. */
  public DriveForwardsTime(DifferentialDriveWrapper drivetrain, long duration, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    this.duration = duration;
    this.speed = speed;
    startTime = System.currentTimeMillis();

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Drive forwards
    m_drivetrain.set(speed, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end (boolean interrupted) {
    m_drivetrain.set(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - startTime) >= duration;
  }
}
