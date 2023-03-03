// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.MotionParameters.Autobalance;
import frc.robot.subsystems.DifferentialDriveWrapper;
import frc.robot.subsystems.Gyro;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalance extends ProfiledPIDCommand {
  // Create our gyro object like we would on a real robot.

  /** Creates a new AutoBalance command. */
  public AutoBalance(Gyro gyro, DifferentialDriveWrapper drivetrain) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            Autobalance.k_p,
            Autobalance.k_i,
            Autobalance.k_d,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(Autobalance.k_maxVelocity, Autobalance.k_maxAcceleration)),
        // This should return the measurement
        () -> gyro.gyro.getAngle(),
        // This should return the goal (can also be a constant)
        // We need to have an angle of 0 degrees and a velocity of 0
        () -> new TrapezoidProfile.State(0, 0),
        // This uses the output
        (output, setpoint) -> {
          SmartDashboard.putNumber("Autobalance.output", output);
          // Use the output (and setpoint, if desired) here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(drivetrain, gyro);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
