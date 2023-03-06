// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.testSubsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SparkMaxTester extends SubsystemBase {
  private int m_motorId = -1;
  private CANSparkMax m_motor;
  /** Creates a new SparkMaxTester. */
  public SparkMaxTester() {
    // Show the default motor id
    SmartDashboard.putNumber("Motor id", -1);
  }

  @Override
  public void periodic() {
    // Change the motor id if the motor has been changed
    int new_id = (int) SmartDashboard.getNumber("Motor id", -1);
    
    // If switching to a new motor
    if (new_id != m_motorId) {
      m_motorId = new_id;
      // Stop and destroy the old motor if possible
      if (m_motor != null) {
        m_motor.set(0);
        m_motor.close();
      }
      // Create a new motor if possible, or make it null
      if (m_motorId < 0) m_motor = null;
      else m_motor = new CANSparkMax(m_motorId, MotorType.kBrushless);
    }

    // Do not do anything if a motor is not selected
    if (m_motor == null) return;
    
    // Print useful data
    SmartDashboard.putNumber("Speed", m_motor.get());
  }
}
