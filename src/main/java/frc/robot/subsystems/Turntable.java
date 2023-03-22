// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turntable extends SubsystemBase {
  private TalonSRX motor;

  /** Creates a new Turntable. */
  public Turntable() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.motor = new TalonSRX(Constants.Ports.k_turntableMotor);

    /* Factory Default all hardware to prevent unexpected behaviour */
    motor.configFactoryDefault();
        
    /* Config the sensor used for Primary PID and sensor direction */
    motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
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

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be
     * neutral within this range. See Table in Section 17.2.1 for native
     * units per rotation.
     */
    motor.configAllowableClosedloopError(0, 0, Constants.MotionParameters.Turntable.k_timeoutMs);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
    motor.config_kF(0, Constants.MotionParameters.Turntable.k_f, Constants.MotionParameters.Turntable.k_timeoutMs);
    motor.config_kP(0, Constants.MotionParameters.Turntable.k_p, Constants.MotionParameters.Turntable.k_timeoutMs);
    motor.config_kI(0, Constants.MotionParameters.Turntable.k_i, Constants.MotionParameters.Turntable.k_timeoutMs);
    motor.config_kD(0, Constants.MotionParameters.Turntable.k_d, Constants.MotionParameters.Turntable.k_timeoutMs);
  }

  /**
   * Set position of turntable in number of encoder counts (clockwise?/counterclockwise?)
   * @param numCounts
   */
  public void setPosition(double numCounts) {
    motor.set(ControlMode.Position, numCounts);
  }

  /**
   * Get position of turntable in number of encoder counts (clockwise?/counterclockwise?)
   */
  public double getPosition() {
    return motor.getSelectedSensorPosition();
  }
}
