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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
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
  public WPI_TalonSRX m_motorL1, m_motorR1;
  public WPI_VictorSPX m_motorL2, m_motorR2;
  public MotorControllerGroup m_left, m_right;
  public DifferentialDrive m_drive;
  public Gyro m_gyro;

  public Field2d m_field;

  // SensorCollection m_leftSensors;
  // SensorCollection m_rightSensors; 

  public DifferentialDriveKinematics m_kinematics;
  public DifferentialDriveOdometry m_odometry;
  public Pose2d m_pose;

  /** Creates a new DifferentialDriveSubsystem. */
  public DifferentialDriveWrapper(Gyro gyro) {
    m_motorL1 = new WPI_TalonSRX(Ports.k_drivetrainMotorControllerPortL1);
    m_motorL2 = new WPI_VictorSPX(Ports.k_drivetrainMotorControllerPortL2);
    m_motorR1 = new WPI_TalonSRX(Ports.k_drivetrainMotorControllerPortR1);
    m_motorR2 = new WPI_VictorSPX(Ports.k_drivetrainMotorControllerPortR2);

    m_field = new Field2d();

    this.m_gyro = gyro;

    m_left = new MotorControllerGroup(m_motorL1, m_motorL2);
    m_right = new MotorControllerGroup(m_motorR1, m_motorR2);
    m_drive = new DifferentialDrive(m_left, m_right);

    // Only one side needs to be reversed, since the motors on the two sides are
    // facing opposite directions.
    m_left.setInverted(false);
    m_right.setInverted(true);

    // m_leftSensors = m_motorL1.getSensorCollection();
    // m_rightSensors = m_motorR1.getSensorCollection();

    //TODO: set pulse width

    // Create kinematics - quick helper functions for path planner
    m_kinematics = new DifferentialDriveKinematics(Constants.Measurements.Drivetrain.k_trackWidth);

    // Create odometry - manages positon on pitch for autonomous
    m_odometry = new DifferentialDriveOdometry(
      gyro.getAngle(IMUAxis.kZ), // TODO: Check correct axis
      m_motorL1.getSelectedSensorPosition()*Constants.MotionParameters.Drivetrain.k_encoderDistancePerPulse,
      m_motorR1.getSelectedSensorPosition()*Constants.MotionParameters.Drivetrain.k_encoderDistancePerPulse,
      new Pose2d(5.0, 13.5, new Rotation2d()));
  }

  /**
   * Drives the robot with the speed and turn values
   * 
   * @param speed how fast the robot moves
   * @param turn  how much the robot turns
   */
  public void drive(double speed, double turn) {
    // Set the deadband for manual driving
    m_drive.setDeadband(DifferentialDrive.kDefaultDeadband);
    m_drive.curvatureDrive(speed, turn, true);
    SmartDashboard.putNumber("Drivetrain.RightPower", m_right.get());
    SmartDashboard.putNumber("Drivetrain.LightPower", m_left.get());
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

  private void resetOdometry(Pose2d initialPose) {
    Rotation2d gyroAngle = m_gyro.getAngle(IMUAxis.kZ); // TODO: Check axis

    // Reset enoder counts
    m_motorL1.setSelectedSensorPosition(0);
    m_motorR1.setSelectedSensorPosition(0);

    m_odometry.resetPosition(gyroAngle,
      0, 0,
      initialPose); // TODO: Check first 3 params
  }
  private Pose2d getPose() {
    return m_pose;
  }
  private DifferentialDriveWheelSpeeds getWheelSpeeds() {
    // Gets encoder values from SensorCollections; https://usermanual.wiki/Pdf/Talon20SRX20Victor20SPX2020Software20Reference20Manual.1959439090.pdf (p128) / https://www.chiefdelphi.com/t/talon-ctre-encoder-values/164553 / https://www.chiefdelphi.com/t/using-encoder-with-talon-srx/145483
    double leftSpeed = m_motorL1.getSelectedSensorVelocity()*10*Constants.MotionParameters.Drivetrain.k_encoderDistancePerPulse;
    double rightSpeed = m_motorR1.getSelectedSensorVelocity()*10*Constants.MotionParameters.Drivetrain.k_encoderDistancePerPulse;
    return new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed);
  }

  private void outputVolts(double vLeft, double vRight) { 
    SmartDashboard.putNumber("drive.vLeft", vLeft);
    SmartDashboard.putNumber("drive.vRight", vRight);
    SmartDashboard.putNumber("drive.posLeft", -m_motorL1.getSelectedSensorPosition());
    SmartDashboard.putNumber("drive.posRight", m_motorR1.getSelectedSensorPosition());
    // Reset the deadband to 0 for autonomous driving
    m_drive.setDeadband(0);
    
    // Use this and not setVoltage to feed the watchdog in the Differential drive
    // This converts voltages to powers
    double leftPower = vLeft / RobotController.getBatteryVoltage();
    double rightPower = vRight / RobotController.getBatteryVoltage();

    // Update field view
    m_field.setRobotPose(m_odometry.getPoseMeters());
    SmartDashboard.putData("Field", m_field);

    m_drive.tankDrive(leftPower, rightPower, false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("WheelSpeeds", this.getWheelSpeeds().toString()); // TODO: Remove

    // Get the rotation of the robot from the gyro.
    Rotation2d gyroAngle = m_gyro.getAngle(IMUAxis.kZ); // TODO: Check axis

    // Update the pose
    m_pose = m_odometry.update(gyroAngle,
      m_motorL1.getSelectedSensorPosition()*Constants.MotionParameters.Drivetrain.k_encoderDistancePerPulse,
      m_motorR1.getSelectedSensorPosition()*Constants.MotionParameters.Drivetrain.k_encoderDistancePerPulse);
  }
}
