// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeGrab extends CommandBase {

  private Intake m_intake;
  private long startTime;
  private boolean hasPeaked = false;
  private long peakTime;


  /** Creates a new IntakeGrab. */
  public IntakeGrab(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    this.m_intake = intake;
    
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.startTime = System.currentTimeMillis();
    this.hasPeaked = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.m_motor.set(ControlMode.PercentOutput, 0.5);
    SmartDashboard.putBoolean("Intake/isMoving", true);

    // Output motor current
    SmartDashboard.putBoolean("Intake/HasPeaked", hasPeaked);
    SmartDashboard.putNumber("Intake/StartTime", startTime);
    SmartDashboard.putNumber("Intake/PeakTime", peakTime);
    SmartDashboard.putNumber("Intake/InputCurrent", m_intake.m_motor.getSupplyCurrent());
    SmartDashboard.putNumber("Intake/OutputCurrent", m_intake.m_motor.getStatorCurrent());

    if (!this.hasPeaked && m_intake.m_motor.getStatorCurrent() > 20 && (System.currentTimeMillis() - startTime) >= 1000) {
      // Peak now
      hasPeaked = true;
      peakTime = System.currentTimeMillis();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Intake/isMoving", false);
    m_intake.m_motor.set(ControlMode.PercentOutput, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Ignore initial peak in current
    return this.hasPeaked && (System.currentTimeMillis() - peakTime) >= 250; // Wait for 250ms after peak
  }
}
