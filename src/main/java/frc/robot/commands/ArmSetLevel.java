// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.dataStorageClasses.ArmPositionCounts;
import frc.robot.subsystems.Arm;

public class ArmSetLevel extends CommandBase {
  private Arm m_arm;
  private int m_positionIndex;

  /** Creates a new ArmSetLevel. */
  public ArmSetLevel(Arm arm, int positionIndex) {
    addRequirements(arm);

    m_arm = arm;
    m_positionIndex = positionIndex;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ArmPositionCounts[] allPositions = Constants.Arm.k_armMotorPositionCounts;
    try {
      // Find the desired positions and set the arm position to that
      ArmPositionCounts position = allPositions[m_positionIndex];
      m_arm.chainMotorDesiredPosition = position.chainMotorCounts;
      SmartDashboard.putString("Arm position", position.name);
    } catch (Exception e){
      // Report the exception, do not fail
      SmartDashboard.putString("Arm errors", "Unknown arm position index");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Exit ASAP
    return true;
  }
}
