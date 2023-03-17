package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Ports;

public class DifferentialDriveWrapper extends SubsystemBase {
  public WPI_VictorSPX m_motorL1 = new WPI_VictorSPX(Ports.k_drivetrainMotorControllerPortL1);
  public WPI_VictorSPX m_motorR1 = new WPI_VictorSPX(Ports.k_drivetrainMotorControllerPortR1);
  public WPI_VictorSPX m_motorL2 = new WPI_VictorSPX(Ports.k_drivetrainMotorControllerPortL2);
  public WPI_VictorSPX m_motorR2 = new WPI_VictorSPX(Ports.k_drivetrainMotorControllerPortR2);;

  public MotorControllerGroup m_left = new MotorControllerGroup(m_motorL1, m_motorL2);
  public MotorControllerGroup m_right = new MotorControllerGroup(m_motorR1, m_motorR2);
  public DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);

  private final Encoder m_leftEncoder = new Encoder(
    Constants.Ports.k_encoderPortAL,
    Constants.Ports.k_encoderPortBL,
    false);
  private final Encoder m_rightEncoder = new Encoder(
    Constants.Ports.k_encoderPortAR,
    Constants.Ports.k_encoderPortBR,
    true);

    // Create kinematics - used to calculate powers based on drive configuration
  public DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Constants.Measurements.Drivetrain.k_trackWidth);

  /** Creates a new DifferentialDriveSubsystem. */
  public DifferentialDriveWrapper() {
    // Only one side needs to be reversed, since the motors on the two sides are
    // facing opposite directions.
    // NOTE: Make sure to keep this the same as the sysid tool and encoder reverse direction flag
    m_left.setInverted(false);
    m_right.setInverted(true);

    
    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(Constants.MotionParameters.Drivetrain.k_distancePerEncoderPulse);
    m_rightEncoder.setDistancePerPulse(Constants.MotionParameters.Drivetrain.k_distancePerEncoderPulse);
  }

  /**
   * Drives the robot with the speed and turn values
   * 
   * @param speed how fast the robot moves
   * @param turn  how much the robot turns
   */
  public void drive(double speed, double turn) {
    m_drive.curvatureDrive(speed, turn, true);
    SmartDashboard.putNumber("drive.RightPower", m_right.get());
    SmartDashboard.putNumber("drive.LightPower", m_left.get());
  }

  /**
   * Outputs the voltages specified into the drivetrain
   * @param vLeft Left voltage, V
   * @param vRight Right voltage, V
   */
  public void outputVolts(double vLeft, double vRight) {
    m_left.setVoltage(vLeft);
    m_right.setVoltage(vRight);
    m_drive.feed();
  }

  /**
   * Sets the speeds specified for each side on the drivetrain
   * @param left Left speed, -1 to 1
   * @param right Right speed, -1 to 1
   */
  public void set(double left, double right) {
    m_left.set(left);
    m_right.set(right);
    m_drive.feed();
  }

  /* Encoders */

  public void resetEncoders(){
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getLeftEncoderDistance() {
    return m_leftEncoder.getDistance();
  }

  public double getRightEncoderDistance() {
    return m_rightEncoder.getDistance();
  }

  public double getLeftEncoderRate() {
    return m_leftEncoder.getRate();
  }

  public double getRightEncoderRate() {
    return m_rightEncoder.getRate();
  }
}
