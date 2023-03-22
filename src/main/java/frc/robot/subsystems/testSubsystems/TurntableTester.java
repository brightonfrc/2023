// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.testSubsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Turntable;

public class TurntableTester extends SubsystemBase {

  Turntable turntable;

  /** Creates a new TurntableTester. */
  public TurntableTester(Turntable turntable) {
    this.turntable = turntable;

    // Show the default num counts
    SmartDashboard.putNumber("TurntableTester/Position", 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double position = SmartDashboard.getNumber("TurntableTester/Position", 0);
    turntable.setPosition((int)position);
  }
}
