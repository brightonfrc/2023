// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.DifferentialDriveWrapper;
import frc.robot.subsystems.Gyro;

public class TurnOnSpot extends CommandBase { 
  DifferentialDriveWrapper m_drivetrain;
  Gyro m_gyro;

  double startAngle;
  double targetAngle;

  double changeInHeading;

  PIDCommand pid;

  /** Creates a new TurnOnSpot. */
  public TurnOnSpot(DifferentialDriveWrapper drivetrain, Gyro gyro, double changeInHeading) { // heading in degrees
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;
    m_gyro = gyro;
    this.changeInHeading = changeInHeading;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startAngle = getAngle();
    targetAngle = startAngle + changeInHeading;

    // Wrap Around
    targetAngle = ((targetAngle + 180) % 360) - 180;

    pid = new PIDCommand(
      new PIDController(Constants.MotionParameters.TurnOnSpot.k_p, Constants.MotionParameters.TurnOnSpot.k_i, Constants.MotionParameters.TurnOnSpot.k_d),
      this::getAngle,
      targetAngle,
      output -> m_drivetrain.m_drive.arcadeDrive(0, output),
      m_drivetrain
    );
    // As angle
    // Set the controller to be continuous (because it is an angle controller)
    pid.getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    pid.getController().setTolerance(Constants.MotionParameters.TurnOnSpot.k_turnToleranceDeg, Constants.MotionParameters.TurnOnSpot.k_turnRateToleranceDegPerS);
  }

  public double getAngle() {
    return m_gyro.getAngle(IMUAxis.kZ).getDegrees();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.set(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.getController().atSetpoint();
  }
}
