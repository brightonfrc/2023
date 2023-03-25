// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.testSubsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.dataStorageClasses.MotorTesterModes;
import frc.robot.subsystems.Arm;

public class ArmTester extends SubsystemBase {
  private XboxController m_controller = new XboxController(Constants.Ports.k_controllerPort);
  private Arm m_arm;

  /** Creates a new ArmTester. */
  public ArmTester() {
    m_arm = new Arm();
    // Show the default motor id
    SmartDashboard.putNumber("Talon Test/Chain Position", m_arm.chainMotorDesiredPosition);
  }

  @Override
  public void periodic() {
    m_arm.chainMotorDesiredPosition = SmartDashboard.getNumber("Talon Test/Chain Position", 0);
  }
}
