// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  public CANSparkMax chainMotor;
  public CANSparkMax cableMotor;
  public SparkMaxPIDController chainMotorPID;
  public SparkMaxPIDController cableMotorPID;
  
  /** Creates a new Arm. */
  public Arm() {
    chainMotor = new CANSparkMax(Constants.Ports.k_armChainMotor, MotorType.kBrushless);
    cableMotor = new CANSparkMax(Constants.Ports.k_armCableMotor, MotorType.kBrushless);

    // NOTE: Use the rev tuner to set PID values
    cableMotorPID = cableMotor.getPIDController();
    chainMotorPID = chainMotor.getPIDController();
  }
}
