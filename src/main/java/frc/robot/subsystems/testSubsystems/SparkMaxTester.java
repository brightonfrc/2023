// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.testSubsystems;

import java.util.HashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.dataStorageClasses.SparkMaxTesterModes;

public class SparkMaxTester extends SubsystemBase {
  private SendableChooser<CANSparkMax> m_motors = new SendableChooser<>();
  private SendableChooser<SparkMaxTesterModes> m_modes = new SendableChooser<>();
  /** Creates a new SparkMaxTester. */
  public SparkMaxTester() {
    // Show the default motor id
    SmartDashboard.putNumber("Test.value", 0);
    m_motors.setDefaultOption("30", new CANSparkMax(30, MotorType.kBrushless));
    m_motors.addOption("31", new CANSparkMax(31, MotorType.kBrushless));
    
    m_modes.setDefaultOption("Percent", SparkMaxTesterModes.Percent);
    m_modes.addOption("SmartMotion", SparkMaxTesterModes.SmartMotion);

    SmartDashboard.putNumber("Test.motor id", m_motors.getSelected().getDeviceId());
  }

  @Override
  public void periodic() {
    var motor = m_motors.getSelected();
    var pid = motor.getPIDController();
    double value = SmartDashboard.getNumber("Test.value", 0);
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
