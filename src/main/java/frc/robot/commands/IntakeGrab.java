// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;

public class IntakeGrab extends CommandBase {

  private Intake m_intake;
  private LED m_led;
  private long startTime;
  private boolean hasPeaked = false;
  private long peakTime;
  private XboxController controller;

  /** Creates a new IntakeGrab. */
  public IntakeGrab(Intake intake, LED led) {
    // Use addRequirements() here to declare subsystem dependencies.
    

    this.controller = new XboxController(Constants.Ports.k_controllerPort);
    this.m_intake = intake;
    this.m_led = led;
    
    addRequirements(intake, led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.startTime = System.currentTimeMillis();
    this.hasPeaked = false;
    
    // Start rumble
    controller.setRumble(RumbleType.kBothRumble, 0.5);
    m_led.set(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.set(0.5);

    // Output motor current
    SmartDashboard.putBoolean("Intake/HasPeaked", hasPeaked);
    SmartDashboard.putNumber("Intake/StartTime", startTime);
    SmartDashboard.putNumber("Intake/PeakTime", peakTime);
    SmartDashboard.putNumber("Intake/InputCurrent", m_intake.getSupplyCurrent());
    SmartDashboard.putNumber("Intake/OutputCurrent", m_intake.getStatorCurrent());

    if (!this.hasPeaked && m_intake.getStatorCurrent() > 20 && (System.currentTimeMillis() - startTime) >= 1000) {
      // Peak now
      hasPeaked = true;
      peakTime = System.currentTimeMillis();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Intake/isMoving", false);
    // End rumble
    controller.setRumble(RumbleType.kBothRumble, 0);
    m_intake.set(0);
    m_led.set(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Ignore initial peak in current
    return this.hasPeaked && (System.currentTimeMillis() - peakTime) >= 250; // Wait for 250ms after peak
  }
}
