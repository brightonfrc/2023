package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

public class Intake extends SubsystemBase {
    private TalonSRX m_motor = new TalonSRX(Ports.k_intakeMotor);

    public Intake() {
        // To lock in place when holding
        m_motor.setNeutralMode(NeutralMode.Brake);
    }
    
    public void set(double value) {
        m_motor.set(ControlMode.PercentOutput, value);
    }
    
    public double getSupplyCurrent() {
        return m_motor.getSupplyCurrent();
    }

    public double getStatorCurrent() {
        return m_motor.getStatorCurrent();
    }
}
