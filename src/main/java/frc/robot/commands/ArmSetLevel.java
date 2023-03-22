// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.dataStorageClasses.ArmPositionCounts;
import frc.robot.dataStorageClasses.TurntablePositionCounts;
import frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;


public class ArmSetLevel extends CommandBase {
  private Arm m_arm;
  private double m_chainMotorDesiredPosition;
  private double m_cableMotorDesiredPosition;
  
  private double positionTolerance = 0.1;
  private double velocityTolerance = 0.1;

  /** Creates a new ArmSetLevel. */
  public ArmSetLevel(Arm arm, int positionIndex) {
    addRequirements(arm);

    m_arm = arm;

    ArmPositionCounts[] allPositions = Constants.ArmPositions.k_armMotorPositionCounts;
    try {
      // Find the desired positions and set the arm position to that
      ArmPositionCounts position = allPositions[positionIndex];
      m_chainMotorDesiredPosition = position.chainMotorCounts;
      m_cableMotorDesiredPosition = position.cableMotorCounts;
      SmartDashboard.putString("Arm position", position.name);
    } catch (Exception e){
      // Report the exception, do not fail
      SmartDashboard.putString("Arm errors", "Unknown arm position index");
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Keep on moving the motor to that position
    m_arm.chainMotorPID.setReference(m_chainMotorDesiredPosition, CANSparkMax.ControlType.kSmartMotion);
    m_arm.cableMotorPID.setReference(m_cableMotorDesiredPosition, CANSparkMax.ControlType.kSmartMotion);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isMotorPIDFinished(m_arm.cableMotor, m_cableMotorDesiredPosition) && isMotorPIDFinished(m_arm.chainMotor, m_chainMotorDesiredPosition);
  }
  
  private boolean isMotorPIDFinished(CANSparkMax motor, double desiredPosition) {
    var encoder = motor.getEncoder();
    double position = encoder.getPosition();
    double velocity = encoder.getVelocity();
    
    return Math.abs(position) < positionTolerance && Math.abs(velocity) < velocityTolerance;
  }
}
