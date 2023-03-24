// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DifferentialDriveWrapper;
import frc.robot.subsystems.Gyro;

public class AutoBalanceV2 extends CommandBase {
  
  private final Gyro gyro;
  private final DifferentialDriveWrapper drivetrain;
  private LinearFilter gyroAngleFilter;
  private LinearFilter gyroSpeedFilter;
  private int gyroAngleFilterSampleNum = 5;
  private int gyroSpeedFilterSampleNum = 5;
  
  // Angular velocity in deg/s to be considered stationary
  private final double k_gyroSpeedThreshold = 5;
  private final double k_gyroAngleThreshold = 15;
  
  private final double k_firstApproachSpeed = 0.3;
  private final double k_stationaryPlaneClimbSpeed = 0.25;
  private final double k_correctionSpeed = 0.2;
  
  private boolean isOnChargingStation;

  /** Creates a new AutoBalanceV2. */
  public AutoBalanceV2(Gyro gyro, DifferentialDriveWrapper drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    
    this.gyro = gyro;
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gyroAngleFilter = LinearFilter.movingAverage(gyroAngleFilterSampleNum);
    gyroSpeedFilter = LinearFilter.movingAverage(gyroSpeedFilterSampleNum);
    
    isOnChargingStation = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = getDesiredSpeed();
    drivetrain.set(speed, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
  // Like Math.signum, but if below the threshold, 0
  private double mathSignThreshold(double threshold, double value) {
    if (Math.abs(value) < threshold) return 0;
    return Math.signum(value);
  }
  
  private double getDesiredSpeed() {
    double gyroAngle = gyroAngleFilter.calculate(-gyro.getAngle(IMUAxis.kY).getDegrees());
    double gyroSpeed = gyroSpeedFilter.calculate(-gyro.getSpeed(IMUAxis.kY).getDegrees());
    
    // Drive differently depending on the "zones"
    // 0 if considered static, -1 if moving back, 1 if forwards
    double angularSpeedSign = mathSignThreshold(k_gyroSpeedThreshold, gyroSpeed);
    double angleSign = mathSignThreshold(k_gyroAngleThreshold, gyroAngle);
    
    // If on the ramp, disable the initial runup
    if (angleSign != 0) isOnChargingStation = true;
    
    // Drive forwards
    if(!isOnChargingStation) 
      return k_firstApproachSpeed;

    // If the angle is 0, we are at the top
    if (angleSign == 0) {
      return 0;
    }
    
    // If we are moving up the ramp and
    // the ramp is still, keep moving
    if (angularSpeedSign == 0) return angleSign * k_stationaryPlaneClimbSpeed;
    // We are starting to tilt
    else {
      // If we are tilting in the opposite direction of driving, break
      if (angleSign == -angularSpeedSign) return 0;
      // else, go back
      else return angleSign * k_correctionSpeed;
    }
  }
}
