package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Ports;

public class DifferentialDriveWrapper extends SubsystemBase {
  public WPI_TalonSRX m_motorL1 = new WPI_TalonSRX(Ports.k_drivetrainMotorControllerPortL1);
  public WPI_TalonSRX m_motorR1 = new WPI_TalonSRX(Ports.k_drivetrainMotorControllerPortR1);
  public WPI_VictorSPX m_motorL2 = new WPI_VictorSPX(Ports.k_drivetrainMotorControllerPortL2);
  public WPI_VictorSPX m_motorR2 = new WPI_VictorSPX(Ports.k_drivetrainMotorControllerPortR2);;

  public MotorControllerGroup m_left = new MotorControllerGroup(m_motorL1, m_motorL2);
  public MotorControllerGroup m_right = new MotorControllerGroup(m_motorR1, m_motorR2);
  public DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);

  public Field2d m_field = new Field2d();

  public Gyro m_gyro;
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
    // Create odometry - manages positon on pitch for autonomous
  public DifferentialDriveOdometry m_odometry;

  /** Creates a new DifferentialDriveSubsystem. */
  public DifferentialDriveWrapper(Gyro gyro) {
    this.m_gyro = gyro;
    
    // Only one side needs to be reversed, since the motors on the two sides are
    // facing opposite directions.
    // NOTE: Make sure to keep this the same as the sysid tool and encoder reverse direction flag
    m_left.setInverted(false);
    m_right.setInverted(true);

    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(Constants.MotionParameters.Drivetrain.k_distancePerEncoderPulse);
    m_rightEncoder.setDistancePerPulse(Constants.MotionParameters.Drivetrain.k_distancePerEncoderPulse);
    
    // Reset the encoders before creating the odometry
    resetEncoders();

    m_odometry = new DifferentialDriveOdometry(
        getGyroHeading(),
        m_leftEncoder.getDistance(),
        m_rightEncoder.getDistance());
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

  // -------------------------------------------------------
  // ---------- Odometry and PathPlanner -------------------
  // -------------------------------------------------------

  // Assuming this method is part of a drivetrain subsystem that provides the
  // necessary methods
  public CommandBase followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if (isFirstPath) {
            this.resetOdometry(traj.getInitialPose());
          }
        }),
        new PPRamseteCommand(
            traj,
            this::getPose, // Pose supplier
            new RamseteController(),
            new SimpleMotorFeedforward(Constants.MotionParameters.Drivetrain.k_s, Constants.MotionParameters.Drivetrain.k_v, Constants.MotionParameters.Drivetrain.k_a),
            this.m_kinematics, // DifferentialDriveKinematics
            this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
            new PIDController(Constants.MotionParameters.Drivetrain.k_p, Constants.MotionParameters.Drivetrain.k_i, Constants.MotionParameters.Drivetrain.k_d), // Left controller. Tune these values for your robot. Leaving them 0 will only
            // use feedforwards.
            new PIDController(Constants.MotionParameters.Drivetrain.k_p, Constants.MotionParameters.Drivetrain.k_i, Constants.MotionParameters.Drivetrain.k_d), // Right controller (usually the same values as left controller)
            this::outputVolts, // Voltage biconsumer
            true, // Should the path be automatically mirrored depending on alliance color.
                  // Optional, defaults to true
            this // Requires this drive subsystem
        ));
  }

  private void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(getGyroHeading(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
  }
  private void resetEncoders(){
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }
  private Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  private DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  private void outputVolts(double vLeft, double vRight) { 
    // Update field view
    m_field.setRobotPose(m_odometry.getPoseMeters());
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putNumber("autoDrive.vLeft", vLeft);
    SmartDashboard.putNumber("autoDrive.vRight", vRight);
    SmartDashboard.putNumber("autoDrive.posLeft", m_leftEncoder.getDistance());
    SmartDashboard.putNumber("autoDrive.posRight", m_rightEncoder.getDistance());
    SmartDashboard.putString("autoDrive.wheelSpeeds", this.getWheelSpeeds().toString());

    m_left.setVoltage(vLeft);
    m_right.setVoltage(vRight);
    m_drive.feed();
  }

  @Override
  public void periodic() {
    // Update the pose
      m_odometry.update(getGyroHeading(),
      m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }
  
  private Rotation2d getGyroHeading(){
    return m_gyro.getAngle(IMUAxis.kZ); // TODO: Check axis    
  }
}
