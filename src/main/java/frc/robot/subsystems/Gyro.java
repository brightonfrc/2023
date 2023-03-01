// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A wrapper for the ADIS gyro */
public class Gyro extends SubsystemBase {
  // Create our gyro object like we would on a real robot.
  public ADIS16470_IMU gyro = new ADIS16470_IMU();
    
    /**
     * Create a new wrapper for a ADIS16470_IMU gyro
     * @param gyro
     */
  public Gyro(ADIS16470_IMU gyro) {
        this.gyro = gyro;
  }

  public Rotation2d getAngle(IMUAxis axis) {
      // Set the axis
      // Note that the gyro does not change the axis if the new axis is the same as the old one, there is no overhead to this
      gyro.setYawAxis(axis);
      // Get the angle
      return new Rotation2d(Math.toRadians(gyro.getAngle()));
  }
}
