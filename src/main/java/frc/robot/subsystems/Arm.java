// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  public CANSparkMax chainMotor;
  public SparkMaxPIDController chainMotorPID;
  public RelativeEncoder chainMotorEncoder;
  public double chainMotorDesiredPosition;
  
  double acceptableError;
  
  /** Creates a new Arm. */
  public Arm(double acceptableError) {
    chainMotor = new CANSparkMax(Constants.Ports.k_armChainMotor, MotorType.kBrushless);

    // NOTE: Ideally, use the rev tuner to set PID values
    chainMotorPID = chainMotor.getPIDController();
    chainMotorEncoder = chainMotor.getEncoder();

    // Reset the encoder at the beginning
    // chainMotor.getEncoder().setPosition(0);

    this.acceptableError = acceptableError;
  }
  
  @Override
  public void periodic() {
    // Keep on moving the motor to that position
    chainMotorPID.setReference(chainMotorDesiredPosition, ControlType.kSmartMotion);

    SmartDashboard.putNumber("Arm/Chain Desired Pos", chainMotorDesiredPosition);

    SmartDashboard.putNumber("Arm/Chain Pos", chainMotorEncoder.getPosition());
  }

  public void resetEncoders() {
    chainMotorEncoder.setPosition(0);
  }
  
  public boolean isArmSettled() {
    double error = chainMotorEncoder.getPosition() - chainMotorDesiredPosition;
    return Math.abs(error) < acceptableError;
  }
}
