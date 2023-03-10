// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.dataStorageClasses.ArmPositionCounts;

public class Arm extends SubsystemBase {
  CANSparkMax m_chainMotor;
  CANSparkMax m_cableMotor;
  SparkMaxPIDController m_chainMotorPID;
  SparkMaxPIDController m_cableMotorPID;
  
  double m_chainMotorDesiredPosition;
  double m_cableMotorDesiredPosition;

  /** Creates a new Arm. */
  public Arm() {
    m_chainMotor = new CANSparkMax(Constants.Ports.k_armChainMotor, MotorType.kBrushless);
    m_cableMotor = new CANSparkMax(Constants.Ports.k_armCableMotor, MotorType.kBrushless);

    // NOTE: Use the rev tuner to set PID values
    m_cableMotorPID = m_cableMotor.getPIDController();
    m_chainMotorPID = m_chainMotor.getPIDController();
    
    setArmPosition(0);
  }
  
  /**
   * Sets the arm to a position using it index
   * @param positionIndex the index of the position in the Constants file
   */
  public void setArmPosition(int positionIndex) {
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

  @Override
  public void periodic() {
    // Keep on moving the motor to that position
    m_chainMotorPID.setReference(m_chainMotorDesiredPosition, CANSparkMax.ControlType.kSmartMotion);
    m_cableMotorPID.setReference(m_cableMotorDesiredPosition, CANSparkMax.ControlType.kSmartMotion);
  }
}
