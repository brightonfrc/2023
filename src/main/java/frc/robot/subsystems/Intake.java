package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

public class Intake extends SubsystemBase {
    private TalonSRX m_motor = new TalonSRX(Ports.k_intakeMotor);

    @Override
    public void periodic() {
        m_motor.set(ControlMode.Velocity, 0.5);
        
        // Output motor current
        SmartDashboard.putNumber("Intake/InputCurrent", m_motor.getSupplyCurrent());
        SmartDashboard.putNumber("Intake/OutputCurrent", m_motor.getStatorCurrent());
    }
}
