// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.testSubsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.dataStorageClasses.MotorTesterModes;

public class TalonSRXTester extends SubsystemBase {
  private SendableChooser<WPI_TalonSRX> m_motors = new SendableChooser<>();
  private SendableChooser<MotorTesterModes> m_modes = new SendableChooser<>();
  private XboxController m_controller = new XboxController(Constants.Ports.k_controllerPort);
  /** Creates a new SparkMaxTester. */
  public TalonSRXTester() {
    // Show the default motor id
    m_motors.setDefaultOption("20", prepareController(20));
    m_motors.addOption("21", prepareController(21));
    
    m_modes.setDefaultOption("Percent", MotorTesterModes.Percent);
    m_modes.addOption("SmartMotion", MotorTesterModes.SmartMotion);
    m_modes.addOption("Position", MotorTesterModes.Position);
    
    SmartDashboard.putNumber("TalonTester/p", 0);
    SmartDashboard.putNumber("TalonTester/i", 0);
    SmartDashboard.putNumber("TalonTester/d", 0);
    SmartDashboard.putNumber("TalonTester/f", 0);

    SmartDashboard.putData("TalonTester/motor", m_motors);
    SmartDashboard.putData("TalonTester/mode", m_modes);
  }
  
  public WPI_TalonSRX prepareController(int id){
    var motor = new WPI_TalonSRX(id);
    motor.configFactoryDefault();

    /* Config the sensor used for Primary PID and sensor direction */
    motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 
                                        0,
                                        Constants.MotionParameters.Turntable.k_timeoutMs);

    /* Ensure sensor is positive when output is positive */
    // motor.setSensorPhase(true);
    // motor.setInverted(false);

    /* Config the peak and nominal outputs, 12V means full */
    motor.configNominalOutputForward(0, Constants.MotionParameters.Turntable.k_timeoutMs);
    motor.configNominalOutputReverse(0, Constants.MotionParameters.Turntable.k_timeoutMs);
    motor.configPeakOutputForward(1, Constants.MotionParameters.Turntable.k_timeoutMs);
    motor.configPeakOutputReverse(-1, Constants.MotionParameters.Turntable.k_timeoutMs);

    motor.configAllowableClosedloopError(0, 0, Constants.MotionParameters.Turntable.k_timeoutMs);

    motor.config_kF(0, Constants.MotionParameters.Turntable.k_f, Constants.MotionParameters.Turntable.k_timeoutMs);
    motor.config_kP(0, Constants.MotionParameters.Turntable.k_p, Constants.MotionParameters.Turntable.k_timeoutMs);
    motor.config_kI(0, Constants.MotionParameters.Turntable.k_i, Constants.MotionParameters.Turntable.k_timeoutMs);
    motor.config_kD(0, Constants.MotionParameters.Turntable.k_d, Constants.MotionParameters.Turntable.k_timeoutMs);
    
    return motor;
  }

  @Override
  public void periodic() {
    var motor = m_motors.getSelected();

    // Reset the encoders if the button is pressed
    if (m_controller.getAButton()) motor.setSelectedSensorPosition(0);
    if (m_controller.getBButton()) {
      double p = SmartDashboard.getNumber("TalonTester/p", 0);
      double i = SmartDashboard.getNumber("TalonTester/i", 0);
      double d = SmartDashboard.getNumber("TalonTester/d", 0);
      double f = SmartDashboard.getNumber("TalonTester/f", 0);
      // Change the PID
      motor.config_kF(0, p, Constants.MotionParameters.Turntable.k_timeoutMs);
      motor.config_kP(0, i, Constants.MotionParameters.Turntable.k_timeoutMs);
      motor.config_kI(0, d, Constants.MotionParameters.Turntable.k_timeoutMs);
      motor.config_kD(0, f, Constants.MotionParameters.Turntable.k_timeoutMs);
    }

    double value = m_controller.getRightY();
    double positionValue = value * 400;
    SmartDashboard.putNumber("TalonTester/value", value);
    SmartDashboard.putNumber("TalonTester/desiredPositionValue", positionValue);
    SmartDashboard.putNumber("TalonTester/encoder_position", motor.getSelectedSensorPosition());
    
    SmartDashboard.putNumber("TalonTester/motor_power", motor.get());

    switch (m_modes.getSelected()) {
      case SmartMotion:
        motor.set(ControlMode.MotionMagic, positionValue);
        return;

      case Position:
      motor.set(ControlMode.Position, positionValue);
        // NOTE: the default is percent
      default:
        motor.set(ControlMode.PercentOutput, value);
        return;
    }
  }
}
