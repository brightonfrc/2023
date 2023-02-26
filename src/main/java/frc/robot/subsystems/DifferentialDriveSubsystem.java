package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ConfigurationPreset;
import frc.robot.Simulation;
import frc.robot.Constants.Measurements;
import frc.robot.Constants.Ports;
import frc.robot.Constants.Strategy;

public class DifferentialDriveSubsystem extends SubsystemBase {
  private MotorControllerGroup m_left, m_right;
  private DifferentialDrive m_drive;
  
  /* Odometry (getting position on field) */
  private Pose2d m_pose;

  private Encoder m_leftEncoder = new Encoder(0, 1);
  private Encoder m_rightEncoder = new Encoder(2, 3);

  // These are our EncoderSim objects, which we will only use in
  // simulation. However, you do not need to comment out these
  // declarations when you are deploying code to the roboRIO.
  private EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  private EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);

  // Create our gyro object like we would on a real robot.
  private AnalogGyro m_gyro = new AnalogGyro(1);

  // Create the simulated gyro object, used for setting the gyro
  // angle. Like EncoderSim, this does not need to be commented out
  // when deploying code to the roboRIO.
  private AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);

  // Create the simulation model of our drivetrain.
  private DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
    DCMotor.getCIM(Measurements.Drivetrain.kMotorsPerCIMGearbox),       // 2 CIM motors on each side of the drivetrain.
    Measurements.Drivetrain.kGearRatio,                    // 7.29:1 gearing reduction.
    Measurements.kMomentOfInertia,                     // MOI of 7.5 kg m^2 (from CAD model).
    Measurements.kMass,                    // The mass of the robot is 60 kg.
    Measurements.Drivetrain.kWheelRadius, // The robot uses 3" radius wheels.
    Measurements.Drivetrain.kTrackWidth,                  // The track width is 0.7112 meters.

    // The standard deviations for measurement noise:
    // x and y:          0.001 m
    // heading:          0.001 rad
    // l and r velocity: 0.1   m/s
    // l and r position: 0.005 m
    VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

  // Creating odometry object to find the position of the robot:
  private DifferentialDriveOdometry m_odometry = null;

  /** Creates a new DifferrentialDriveSubsystem. */
  public DifferentialDriveSubsystem() {
    var m_motorL1 = new WPI_TalonSRX(Ports.k_DrivetrainMotorControllerPortL1);
    var m_motorL2 = new WPI_VictorSPX(Ports.k_DrivetrainMotorControllerPortL2);
    var m_motorR1 = new WPI_TalonSRX(Ports.k_DrivetrainMotorControllerPortR1);
    var m_motorR2 = new WPI_VictorSPX(Ports.k_DrivetrainMotorControllerPortR2);
    
    m_left = new MotorControllerGroup(m_motorL1, m_motorL2);
    m_right = new MotorControllerGroup(m_motorR1, m_motorR2);
    m_drive = new DifferentialDrive(m_left, m_right);

    m_left.setInverted(true);
    m_right.setInverted(false);

    this.initOdometry();
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

  private void initOdometry() {
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * Measurements.Drivetrain.kWheelRadius / Measurements.Drivetrain.kEncoderResolution);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * Measurements.Drivetrain.kWheelRadius / Measurements.Drivetrain.kEncoderResolution);

  }

  /**
   * Resets the internal odometry values to correspond a specific position on the pitch
   * @param startingConfiguration the selected preset configuration for the robot
   */
  public void setStartingPosition(ConfigurationPreset startingConfiguration) {
    // Set odometry pose to starting pose
    Pose2d startingPos = startingConfiguration.isRed ? Strategy.kStartRed : Strategy.kStartBlue; 
    m_odometry = new DifferentialDriveOdometry(
      m_gyro.getRotation2d(),
      m_leftEncoder.getDistance(), m_rightEncoder.getDistance(),
      startingPos);
  }

  @Override
  public void simulationPeriodic() {
    double leftPowerFraction = m_left.get();
    double rightPowerFraction = m_right.get();
    
    // Set the inputs to the system. Note that we need to convert
    // the [-1, 1] PWM signal to voltage by multiplying it by the
    // robot controller voltage.
    m_driveSim.setInputs(leftPowerFraction * RobotController.getInputVoltage(),
    rightPowerFraction * RobotController.getInputVoltage());

    // Advance the model by 20 ms. Note that if you are running this
    // subsystem in a separate thread or have changed the nominal timestep
    // of TimedRobot, this value needs to match it.
    m_driveSim.update(0.02);

    // Update all of our sensors.
    m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());

    m_odometry.update(m_gyro.getRotation2d(),
    m_leftEncoder.getDistance(),
    m_rightEncoder.getDistance());
    m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());

    SmartDashboard.putNumber("Simulation.Drivetrain.LeftPos (m)", m_leftEncoder.getDistance());
    SmartDashboard.putNumber("Simulation.Drivetrain.LeftSpeed (m/s)", m_leftEncoder.getRate());
    SmartDashboard.putNumber("Simulation.Drivetrain.RightPos (m)", m_rightEncoder.getDistance());
    SmartDashboard.putNumber("Simulation.Drivetrain.RightSpeed (m/s)", m_rightEncoder.getRate());

    Simulation.field.setRobotPose(m_odometry.getPoseMeters());
  }

  @Override
  public void periodic() {
    // Update odometry
    m_pose = m_odometry.update(m_gyro.getRotation2d(),
    m_leftEncoder.getDistance(),
    m_rightEncoder.getDistance());

    SmartDashboard.putString("Drivetrain.Pose", m_odometry.getPoseMeters().toString());
  }
}
