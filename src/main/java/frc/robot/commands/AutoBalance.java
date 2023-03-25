// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.MotionParameters;
import frc.robot.subsystems.DifferentialDriveWrapper;
import frc.robot.subsystems.Gyro;


/**
 * A command for driving up the charge station and balancing on top of it. Thanks to https://www.chiefdelphi.com/t/psa-balance-in-auto/ .
 */
public class AutoBalance extends CommandBase {
  /** 
   * The Gyro
   */
  private Gyro m_gyro;
  /** 
   * The Drivetrain
   */
  private DifferentialDriveWrapper m_drivetrain;
  /** 
   * The current state of the autonomous routine
   * For pure balance
   * > 0 = drive forwards to approach station
   * > 1 = drive up charge station
   * > 2 = on charge station - stop
   * > 3/4 = stopped
   */
  private int state = 0;
  /**
   * Timer from start of state trigger to state entry
   */
  private int debounceCount = 0;

  private boolean isReversed;
  

  public AutoBalance(Gyro gyro, DifferentialDriveWrapper drivetrain, boolean isReversed) {
    addRequirements(drivetrain);

    m_gyro = gyro;
    m_drivetrain = drivetrain;
    this.isReversed = isReversed;
  }


  // returns the magnititude of the robot's tilt calculated by Pythagoras of pitch
  // and roll, used to compensate for diagonally mounted rio
  public double getTilt() {
    double tilt = -m_gyro.getAngle(IMUAxis.kY).getDegrees();

    if(isReversed) {
      tilt *= -1; // Negate tilt so acts like gyro was other way round
    }

    SmartDashboard.putNumber("AutoBalance/State", state);
    SmartDashboard.putNumber("AutoBalance/Tilt", tilt);
    SmartDashboard.putBoolean("AutoBalance/Reversed", isReversed);

    return tilt;
  }

  public int secondsToTicks(double time) {
    return (int) (time * 50);
  }

  // routine for automatically driving onto and engaging the charge station.
  // returns a value from -1.0 to 1.0, which left and right motors should be set
  // to.
  public double autoBalanceRoutine() {
    switch (state) {
        // drive forwards to approach station, exit when tilt is detected
        case 0:
            if (getTilt() > MotionParameters.AutoBalance.k_onChargeStationDegree) {
                debounceCount++;
            }
            if (debounceCount > secondsToTicks(MotionParameters.AutoBalance.k_debounceTime)) {
                state = 1;
                debounceCount = 0;
                return MotionParameters.AutoBalance.k_robotSpeedSlow;
            }
            return MotionParameters.AutoBalance.k_robotSpeedFast;
        // driving up charge station, drive slower, stopping when level
        case 1:
            if (getTilt() < MotionParameters.AutoBalance.k_levelDegree) {
                debounceCount++;
            }
            if (debounceCount > secondsToTicks(MotionParameters.AutoBalance.k_debounceTime)) {
                state = 2;
                debounceCount = 0;
                return 0;
            }
            return MotionParameters.AutoBalance.k_robotSpeedSlow;
        // on charge station, stop motors and wait for end of auto
        case 2:
        if (Math.abs(getTilt()) <= Constants.MotionParameters.AutoBalance.k_levelDegree / 2) {
                debounceCount++;
            }
            if (debounceCount > secondsToTicks(Constants.MotionParameters.AutoBalance.k_debounceTime)) {
                state = 4;
                debounceCount = 0;
                return 0;
            }
            if (getTilt() >=  Constants.MotionParameters.AutoBalance.k_levelDegree) {
                return Constants.MotionParameters.AutoBalance.k_robotSpeedCorrection;
            } else if (getTilt() <= -Constants.MotionParameters.AutoBalance.k_levelDegree) {
                return -Constants.MotionParameters.AutoBalance.k_robotSpeedCorrection;
            }
        case 3:
            return 0;
    }
    return 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = autoBalanceRoutine();
    SmartDashboard.putNumber("AutoBalance/Speed", speed);

    if(isReversed) {
      speed *= -1; // Negate speed so moves backwards
    }
    
    // Try to account for voltage difference due to battery charge
    double desiredVoltage = speed * 12.5;
    m_drivetrain.outputVolts(desiredVoltage, desiredVoltage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("[AutoBalance] End - " + (interrupted ? "Interrupted" : "Not Interrupted"));
    m_drivetrain.set(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return state >= 3;
    return false;
  }
}
