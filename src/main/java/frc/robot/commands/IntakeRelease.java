// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeRelease extends CommandBase {

  private Intake m_intake;
  private long startTime;


  /** Creates a new IntakeGrab. */
  public IntakeRelease(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    this.m_intake = intake;
    
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.set(-1);

    // Output start time
    SmartDashboard.putNumber("Intake/StartTime", startTime);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Ignore initial peak in current
    return System.currentTimeMillis() - startTime >= 500; // Wait for 500ms
  }
}
