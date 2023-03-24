package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

public class Intake extends SubsystemBase {
    public TalonSRX m_motor = new TalonSRX(Ports.k_intakeMotor);

    public Intake() {
        // To lock in place when holding
        m_motor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() {
    }
}
