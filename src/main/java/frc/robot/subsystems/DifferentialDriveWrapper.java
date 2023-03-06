package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

public class DifferentialDriveWrapper extends SubsystemBase {
  public MotorControllerGroup m_left, m_right;
  public DifferentialDrive m_drive;

  public Encoder m_leftEncoder = new Encoder(0, 1);
  public Encoder m_rightEncoder = new Encoder(2, 3);

  /** Creates a new DifferrentialDriveSubsystem. */
  public DifferentialDriveWrapper() {
    var m_motorL1 = new WPI_TalonSRX(Ports.k_drivetrainMotorControllerPortL1);
    var m_motorL2 = new WPI_VictorSPX(Ports.k_drivetrainMotorControllerPortL2);
    var m_motorR1 = new WPI_TalonSRX(Ports.k_drivetrainMotorControllerPortR1);
    var m_motorR2 = new WPI_VictorSPX(Ports.k_drivetrainMotorControllerPortR2);
    
    m_left = new MotorControllerGroup(m_motorL1, m_motorL2);
    m_right = new MotorControllerGroup(m_motorR1, m_motorR2);
    m_drive = new DifferentialDrive(m_left, m_right);

    // Only one side needs to be reversed, since the motors on the two sides are facing opposite directions.
    m_left.setInverted(true);
    m_right.setInverted(false);
  }
  
  /**
   * Drives the robot with the speed and turn values
   * @param speed how fast the robot moves
   * @param turn how much the robot turns
   */
  public void drive(double speed, double turn) {
    m_drive.curvatureDrive(speed, turn, true);
    SmartDashboard.putNumber("Drivetrain.RightPower", m_right.get());
    SmartDashboard.putNumber("Drivetrain.LightPower", m_left.get());
  }
}
