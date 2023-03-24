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

  /** Creates a new ArmManualLevel. */
  public ArmManualLevel(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("Arm Manual/Status", "Initialised");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("Arm Manual/Status", "Running");
    XboxController controller = new XboxController(Constants.Ports.k_controllerPort);
    double speed = controller.getLeftY(); // TODO for now
    if(controller.getBackButton()) {
      // "-" button
      SmartDashboard.putNumber("Arm Manual/Speed", speed);
      SmartDashboard.putString("Arm Manual/Motor", "Cable");
      m_arm.cableMotor.set(speed);
      m_arm.chainMotor.set(0);
    } else {
      SmartDashboard.putNumber("Arm Manual/Speed", speed);
      SmartDashboard.putString("Arm Manual/Motor", "Chain");
      m_arm.chainMotor.set(speed);
      m_arm.cableMotor.set(0);
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
