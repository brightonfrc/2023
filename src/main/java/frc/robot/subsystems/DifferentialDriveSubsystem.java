package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Measurements;
import frc.robot.Constants.Ports;
import frc.robot.Constants.RobotSettings;
import frc.robot.Constants.Strategy;

public class DifferentialDriveSubsystem extends SubsystemBase {
  public TalonSRX m_motorL1;
  public VictorSPX m_motorL2;
  public TalonSRX m_motorR1;
  public VictorSPX m_motorR2;
  
  private boolean isReversed;

  /* Saved for simulation */
  private double leftVoltage;
  private double rightVoltage;
  private BooleanSubscriber isRedAllianceSubscriber;

  /** Creates a new DifferrentialDriveSubsystem. */
  public DifferentialDriveSubsystem(BooleanSubscriber isRedAllianceSubscriber) {
    m_motorL1 = new TalonSRX(Ports.k_DrivetrainMotorControllerPortL1);
    m_motorL2 = new VictorSPX(Ports.k_DrivetrainMotorControllerPortL2);
    m_motorR1 = new TalonSRX(Ports.k_DrivetrainMotorControllerPortR1);
    m_motorR2 = new VictorSPX(Ports.k_DrivetrainMotorControllerPortR2);

    this.isRedAllianceSubscriber = isRedAllianceSubscriber;
    
    this.isReversed = RobotSettings.k_DrivetrainStartInverted;

    m_motorL1.setInverted(true);
    m_motorR1.setInverted(false);
    
    
    m_motorL2.setInverted(InvertType.FollowMaster);
    m_motorR2.setInverted(InvertType.FollowMaster);


    m_motorL2.follow(m_motorL1);
    m_motorR2.follow(m_motorR1);

    // Stop the motors when trying to stop the robot
    m_motorL1.setNeutralMode(NeutralMode.Brake);
    m_motorL2.setNeutralMode(NeutralMode.Brake);
    m_motorR1.setNeutralMode(NeutralMode.Brake);
    m_motorR2.setNeutralMode(NeutralMode.Brake);

    this.initOdometry();
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

    this.leftVoltage = leftPower;
    this.rightVoltage = rightPower;
    
    SmartDashboard.putNumber("Left", leftPower);
    SmartDashboard.putNumber("Right", rightPower);
  }
  
  /**
   * An intuitive way to drive a robot drivetrain
   * @param speed forward speed, -1 to 1 (where negative values move backwards)
   * @param turn how much to turn, -1 to 1 (negative values mean left)
   */
  public void arcadeDrive(double speed, double turn){  
    if(Math.abs(turn) < 0.1) {
      turn = 0;
    }

    double left = speed + turn;
    double right = speed - turn;

    // Telemetry for debugging
    SmartDashboard.putNumber("Drivetrain.left_power", left);
    SmartDashboard.putNumber("Drivetrain.right_power", right);
    setPower(left, right);
  }

  /* vvv Odometry (getting position on field) vvv */

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
  private DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim( // TODO: Update details
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

  // Creating my odometry object to find the position of the robot:
  // our starting pose is 5 meters along the long end of the field and in the
  // center of the field along the short end, facing forward.
  private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
    m_gyro.getRotation2d(),
    m_leftEncoder.getDistance(), m_rightEncoder.getDistance(),
    new Pose2d(0.0, 0.0, new Rotation2d()));

  

  private Pose2d m_pose;
  private Field2d m_field = new Field2d();

  private void initOdometry() {
    // // Create the trajectory to follow in autonomous. It is best to initialize
    // // trajectories here to avoid wasting time in autonomous.
    // Trajectory m_trajectory =
    //     TrajectoryGenerator.generateTrajectory(
    //         new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    //         List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //         m_pose,
    //         new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));

    // // Create and push Field2d to SmartDashboard.
    // Field2d m_field = new Field2d();
    // SmartDashboard.putData(m_field);

    // // Push the trajectory to Field2d.
    // m_field.getObject("traj").setTrajectory(m_trajectory);

    // TODO: Update
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * Measurements.Drivetrain.kWheelRadius / Measurements.Drivetrain.kEncoderResolution);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * Measurements.Drivetrain.kWheelRadius / Measurements.Drivetrain.kEncoderResolution);

    SmartDashboard.putData("Field", m_field);
  }

  public void startingPosition() {
    // Set odometry pose to starting pose - TODO move simulation-specific code to separate subsystem, and make odometry public
    if(isRedAllianceSubscriber.get()) { // Red alliance
      m_odometry.resetPosition(m_gyro.getRotation2d(),
      m_leftEncoder.getDistance(),
      m_rightEncoder.getDistance(), 
      Strategy.kStartRed);
    } else { // Blue alliance
      m_odometry.resetPosition(m_gyro.getRotation2d(),
      m_leftEncoder.getDistance(),
      m_rightEncoder.getDistance(), 
      Strategy.kStartBlue);
    }
  }

  public void simulationPeriodic() {

    SmartDashboard.putNumber("LeftVIn (m/s)", this.leftVoltage * RobotController.getInputVoltage());
    SmartDashboard.putNumber("RightVIn (m/s)", this.rightVoltage * RobotController.getInputVoltage());

    // Set the inputs to the system. Note that we need to convert
    // the [-1, 1] PWM signal to voltage by multiplying it by the
    // robot controller voltage.
    m_driveSim.setInputs(this.leftVoltage * RobotController.getInputVoltage(),
    this.rightVoltage * RobotController.getInputVoltage());

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
    m_field.setRobotPose(m_odometry.getPoseMeters());

    SmartDashboard.putNumber("LeftPos (m)", m_leftEncoder.getDistance());
    SmartDashboard.putNumber("LeftVOut (m/s)", m_leftEncoder.getRate());
    SmartDashboard.putNumber("RightPos (m)", m_rightEncoder.getDistance());
    SmartDashboard.putNumber("RightVOut (m/s)", m_rightEncoder.getRate());

    m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());

    m_field.setRobotPose(m_odometry.getPoseMeters());
    SmartDashboard.putString("Pose", m_odometry.getPoseMeters().toString());
  }

  @Override
  public void periodic() {
    // Get the rotation of the robot from the gyro.
    var gyroAngle = m_gyro.getRotation2d();

    // Update the pose
    m_pose = m_odometry.update(gyroAngle,
    m_leftEncoder.getDistance(),
    m_rightEncoder.getDistance());

    // // This will get the simulated sensor readings that we set 
    // // while in simulation, but will use
    // // real values on the robot itself.
    // m_odometry.update(m_gyro.getRotation2d(),
    // m_leftEncoder.getDistance(),
    // m_rightEncoder.getDistance());
  }
}
