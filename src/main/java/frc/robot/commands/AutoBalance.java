// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DifferentialDriveWrapper;
import frc.robot.subsystems.Gyro;


/**
 * A command for driving up the charge station and balancing on top of it. Thanks to https://www.chiefdelphi.com/t/psa-balance-in-auto/ .
 */
public class AutoBalance extends CommandBase {
  // private SimpleMotorFeedforward drivetrainFeedforward = new SimpleMotorFeedforward(Constants.MotionParameters.Drivetrain.k_s, Constants.MotionParameters.Drivetrain.k_v, Constants.MotionParameters.Drivetrain.k_a);
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
  private int state;
  /**
   * Timer from start of state trigger to state entry
   */
  private int debounceCount;
  /**
   * Robot speed when driving up slope of charge station
   */
  private double robotSpeedSlow;
  /**
   * Robot speed when driving towards charge station
   */
  private double robotSpeedFast;
  /**
   * Minimum Angle at which the robot is driving up the charge station (degrees)
   */
  private double onChargeStationDegree;
  /**
   * Maximum Angle at which the robot is level, on top of the charge station (degrees)
   */
  private double levelDegree;
  /**
   * Time delay from start of state trigger to state entry (seconds)
   */
  private double debounceTime;

  public AutoBalance(Gyro gyro, DifferentialDriveWrapper drivetrain) {
    addRequirements(drivetrain);

    m_gyro = gyro;
    m_drivetrain = drivetrain;

    state = 0;
    debounceCount = 0;

    /**********
     * CONFIG *
     **********/
    // Speed the robot drived while scoring/approaching station, default = 0.4
    robotSpeedFast = 0.4;

    // Speed the robot drives while balancing itself on the charge station.
    // Should be roughly half the fast speed, to make the robot more accurate,
    // default = 0.2
    robotSpeedSlow = 0.2;

    // Angle where the robot knows it is on the charge station, default = 13.0
    onChargeStationDegree = 13.0;

    // Angle where the robot can assume it is level on the charging station
    // Used for exiting the drive forward sequence as well as for auto balancing,
    // default = 6.0
    levelDegree = 6.0;

    // Amount of time a sensor condition needs to be met before changing states in
    // seconds
    // Reduces the impact of sensor noice, but too high can make the auto run
    // slower, default = 0.2
    debounceTime = 0.2;
    // TODO: Construct command
  }


  // returns the magnititude of the robot's tilt calculated by Pythagoras of pitch
  // and roll, used to compensate for diagonally mounted rio
  public double getTilt() {
    // TODO: Check
    // double pitch = m_gyro.getAngle(IMUAxis.kX).getDegrees();
    // double roll = m_gyro.getAngle(IMUAxis.kY).getDegrees();
    // if ((pitch + roll) >= 0) {
    //     return Math.sqrt(pitch * pitch + roll * roll);
    // } else {
    //     return -Math.sqrt(pitch * pitch + roll * roll);
    // }
    return m_gyro.getAngle(IMUAxis.kX).getDegrees();
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
            if (getTilt() > onChargeStationDegree) {
                debounceCount++;
            }
            if (debounceCount > secondsToTicks(debounceTime)) {
                state = 1;
                debounceCount = 0;
                return robotSpeedSlow;
            }
            return robotSpeedFast;
        // driving up charge station, drive slower, stopping when level
        case 1:
            if (getTilt() < levelDegree) {
                debounceCount++;
            }
            if (debounceCount > secondsToTicks(debounceTime)) {
                state = 2;
                debounceCount = 0;
                return 0;
            }
            return robotSpeedSlow;
        // on charge station, stop motors and wait for end of auto
        case 2:
            if (Math.abs(getTilt()) <= levelDegree / 2) {
                debounceCount++;
            }
            if (debounceCount > secondsToTicks(debounceTime)) {
                state = 4;
                debounceCount = 0;
                return 0;
            }
            if (getTilt() >= levelDegree) {
                return 0.1;
            } else if (getTilt() <= -levelDegree) {
                return -0.1;
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
    // speed = drivetrainFeedforward.calculate(speed);
    // Update the wheel speeds - TODO: Check is correct
    m_drivetrain.drive(speed, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return state >= 3;
  }
}
