// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmManualLevel extends CommandBase {
  Arm m_arm;
  private double m_positionChangeRate = 20;

  private long m_lastUpdateTime;

  /** Creates a new ArmManualLevel. */
  public ArmManualLevel(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    addRequirements(arm);

    m_lastUpdateTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    long currentTime = System.currentTimeMillis();
    double positionChange = m_positionChangeRate * (currentTime - m_lastUpdateTime) / 1000;

    XboxController controller = new XboxController(Constants.Ports.k_controllerPort);
    positionChange *= controller.getLeftY();

    m_lastUpdateTime = currentTime;

    SmartDashboard.putString("Arm Manual/Motor", "Chain");
    m_arm.chainMotorDesiredPosition += positionChange;
    SmartDashboard.putNumber("Arm Manual/Speed", positionChange);
    
    // If minus and left back buttons are pressed, reset the encoders
    if (controller.getBackButton() && controller.getLeftStickButton()) {
      m_arm.resetEncoders();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
