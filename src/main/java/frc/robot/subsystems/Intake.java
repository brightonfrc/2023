// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax m_motor1;
  private CANSparkMax m_motor2;

  /** Creates a new Intake. */
  public Intake() {
    m_motor1 = new CANSparkMax(Constants.Ports.k_armChainMotor, MotorType.kBrushless);
    m_motor2 = new CANSparkMax(Constants.Ports.k_armCableMotor, MotorType.kBrushless);
    
   m_motor1.setInverted(true);
   SmartDashboard.putNumber("intake.speed", 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double speed = SmartDashboard.getNumber("intake.speed", 0);
    m_motor1.set(speed);
    m_motor2.set(speed);
  }
}
