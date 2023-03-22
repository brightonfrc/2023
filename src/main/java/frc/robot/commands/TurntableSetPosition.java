// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.dataStorageClasses.TurntablePositionCounts;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Turntable;

import com.revrobotics.CANSparkMax;


public class TurntableSetPosition extends CommandBase {
  private Turntable m_turntable;
  private TurntablePositionCounts position;

  /** Creates a new ArmSetLevel. */
  public TurntableSetPosition(Turntable turntable, int positionIndex) {
    addRequirements(turntable);

    m_turntable = turntable;

    // TODO: Update

    // TurntablePositionCounts[] allPositions = Constants.ArmPositions.k_armMotorPositionCounts;
    // try {
    //   // Find the desired positions and set the arm position to that
    //   position = allPositions[positionIndex];
    //   SmartDashboard.putString("Turntable position", position.name);
    // } catch (Exception e){
    //   // Report the exception, do not fail
    //   SmartDashboard.putString("Turntable errors", "Unknown arm position index");
    // }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turntable.setPosition(position.encoderCounts);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_turntable.getPosition() == position.encoderCounts);
  }
}
