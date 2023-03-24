// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  public CANSparkMax chainMotor;
  public CANSparkMax cableMotor;
  public SparkMaxPIDController chainMotorPID;
  public SparkMaxPIDController cableMotorPID;
  public double chainMotorDesiredPosition;
  public double cableMotorDesiredPosition;
  
  /** Creates a new Arm. */
  public Arm() {
    chainMotor = new CANSparkMax(Constants.Ports.k_armChainMotor, MotorType.kBrushless);
    cableMotor = new CANSparkMax(Constants.Ports.k_armCableMotor, MotorType.kBrushless);

    // chainMotor.getEncoder().setPosition(0);
    // cableMotor.getEncoder().setPosition(0);

    // NOTE: Ideally, use the rev tuner to set PID values
    cableMotorPID = cableMotor.getPIDController();
    chainMotorPID = chainMotor.getPIDController();

    // However, this motor controller doesn't save the values properly
    cableMotorPID.setP(Constants.Arm.cableMotorP);
    cableMotorPID.setI(Constants.Arm.cableMotorI);
    cableMotorPID.setD(Constants.Arm.cableMotorD);
    cableMotorPID.setFF(Constants.Arm.cableMotorFF);
  }
  
  @Override
  public void periodic() {
    // Keep on moving the motor to that position
    chainMotorPID.setReference(chainMotorDesiredPosition, ControlType.kSmartMotion);
    cableMotorPID.setReference(cableMotorDesiredPosition, ControlType.kSmartMotion);

    SmartDashboard.putNumber("Arm/Chain Desired Pos", chainMotorDesiredPosition);
    SmartDashboard.putNumber("Arm/Cable Desired Pos", cableMotorDesiredPosition);

    SmartDashboard.putNumber("Arm/Chain Pos", chainMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Arm/Cable Pos", cableMotor.getEncoder().getPosition());
  }

  public void resetEncoders() {
    cableMotor.getEncoder().setPosition(0);
    chainMotor.getEncoder().setPosition(0);
  }
}
