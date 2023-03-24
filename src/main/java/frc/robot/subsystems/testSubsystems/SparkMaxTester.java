// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.testSubsystems;

import java.util.HashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.dataStorageClasses.MotorTesterModes;

public class SparkMaxTester extends SubsystemBase {
  private SendableChooser<CANSparkMax> m_motors = new SendableChooser<>();
  private SendableChooser<MotorTesterModes> m_modes = new SendableChooser<>();
  private XboxController m_controller = new XboxController(Constants.Ports.k_controllerPort);
  /** Creates a new SparkMaxTester. */
  public SparkMaxTester() {
    // Show the default motor id
    SmartDashboard.putNumber("SparkMax Test/Value", 0);
    m_motors.setDefaultOption("30", new CANSparkMax(30, MotorType.kBrushless));
    m_motors.addOption("31", new CANSparkMax(31, MotorType.kBrushless));
    
    m_modes.setDefaultOption("Percent", MotorTesterModes.Percent);
    m_modes.addOption("SmartMotion", MotorTesterModes.SmartMotion);

    SmartDashboard.putNumber("SparkMax Test/Motor ID", m_motors.getSelected().getDeviceId());
  }

  @Override
  public void periodic() {
    var motor = m_motors.getSelected();
    var encoder = motor.getEncoder();
    // Reset the encoders if the button is pressed
    if (m_controller.getAButton()) encoder.setPosition(0);

    var pid = motor.getPIDController();
    double value = m_controller.getRightY();
    SmartDashboard.putNumber("SparkMaxTester/Value", value);
    SmartDashboard.putNumber("SparkMaxTester/Encoder Position", encoder.getPosition());
    switch (m_modes.getSelected()) {
      case SmartMotion:
        pid.setReference(value, ControlType.kSmartMotion);
        return;
        // NOTE: the default is percent
      default:
        motor.set(value);
        return;
    }
  }
}
