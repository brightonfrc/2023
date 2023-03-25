// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TestCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DifferentialDriveWrapper;

public class TestDrivetrainPID extends CommandBase {
  private DifferentialDriveWrapper drivetrain;
  /** Creates a new TestDrivetrainPID. */
  public TestDrivetrainPID(DifferentialDriveWrapper drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var pid = drivetrain.forwardVelocityPID;
    SmartDashboard.putNumber("testDrive/p", pid.getP());
    SmartDashboard.putNumber("testDrive/i", pid.getI());
    SmartDashboard.putNumber("testDrive/d", pid.getD());

    SmartDashboard.putNumber("testDrive/speed", 0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var pid = drivetrain.forwardVelocityPID;

    double p = SmartDashboard.getNumber("testDrive/p", 0);
    double i = SmartDashboard.getNumber("testDrive/i", 0);
    double d = SmartDashboard.getNumber("testDrive/d", 0);

    pid.setP(p);
    pid.setI(i);
    pid.setD(d);

    double speed = SmartDashboard.getNumber("testDrive/speed", 0);

    drivetrain.setPowerByPID(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
