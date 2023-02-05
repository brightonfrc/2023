package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants.RobotSettings;

public class DifferentialDriveSubsystem extends SubsystemBase {
  public VictorSPX m_motorL1;
  public VictorSPX m_motorL2;
  public VictorSPX m_motorR1;
  public VictorSPX m_motorR2;
  
  private boolean isReversed;

  /** Creates a new DifferrentialDriveSubsystem. */
  public DifferentialDriveSubsystem() {
    m_motorL1 = new VictorSPX(Ports.k_DrivetrainMotorControllerPortL1);
    m_motorL2 = new VictorSPX(Ports.k_DrivetrainMotorControllerPortL2);
    m_motorR1 = new VictorSPX(Ports.k_DrivetrainMotorControllerPortR1);
    m_motorR2 = new VictorSPX(Ports.k_DrivetrainMotorControllerPortR2);
    
    this.isReversed = RobotSettings.k_DrivetrainStartInverted;

    m_motorL1.setInverted(false);
    m_motorR1.setInverted(true);
    
    
    m_motorL2.setInverted(InvertType.FollowMaster);
    m_motorR2.setInverted(InvertType.FollowMaster);


    m_motorL2.follow(m_motorL1);
    m_motorR2.follow(m_motorR1);

    // Stop the motors when trying to stop the robot
    m_motorL1.setNeutralMode(NeutralMode.Brake);
    m_motorL2.setNeutralMode(NeutralMode.Brake);
    m_motorR1.setNeutralMode(NeutralMode.Brake);
    m_motorR2.setNeutralMode(NeutralMode.Brake);
  }
  
  /**
   * Changes whether the drivetrain is reversed or not
   * @param isReversed
   */
  public void setIsReversed(boolean isReversed){
    this.isReversed = isReversed;
  }

  /**
   * Sets the powers of the two sides of the motors, from -1 to 1
   * A power of -1 drives in reverse
   * @param leftPower power of the motors on the left side
   * @param rightPower power of the motors on the right side
   */
  public void setPower(double leftPower, double rightPower){
    if (isReversed) {
      leftPower *= -1;
      rightPower *= -1;
    }
    m_motorL1.set(ControlMode.PercentOutput, leftPower);
    m_motorR1.set(ControlMode.PercentOutput, rightPower);
  }
  
  /**
   * An intuitive way to drive a robot drivetrain
   * @param speed forward speed, -1 to 1 (where negative values move backwards)
   * @param turn how much to turn, -1 to 1 (negative values mean left)
   */
  public void arcadeDrive(double speed, double turn){
    
    double left = speed + turn;
    double right = speed - turn;

    SmartDashboard.putNumber("Left", left);
    SmartDashboard.putNumber("Right", right);
    setPower(left, right);
  }
}
