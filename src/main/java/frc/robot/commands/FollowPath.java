// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DifferentialDriveWrapper;
import frc.robot.subsystems.Gyro;

/**
 * A command to follow a PathPlanner trajectory.
 * This sets up the odometry and gyro, but the encoders are included in the drivetrain.
 */
public class FollowPath extends CommandBase { // TODO: Test
  SequentialCommandGroup sequentialCommandGroup;
  DifferentialDriveWrapper m_drivetrain;

  public Gyro m_gyro;
  // Create odometry - manages position on pitch for autonomous
  public DifferentialDriveOdometry m_odometry;

  /**
   * Creates a new command to follow a path from PathPlanner with the drivetrain.
   * @param drivetrain The DifferentialDriveWrapper drivetrain
   * @param gyro The Gyroscope to use for odometry
   * @param traj The PathPlannerTrajectory to follow
   */
  public FollowPath(DifferentialDriveWrapper drivetrain, Gyro gyro, PathPlannerTrajectory traj) {
    this.m_drivetrain = drivetrain;
    this.m_gyro = gyro;
    
    // Use addRequirements() here to declare subsystem dependencies.
    this.sequentialCommandGroup = new SequentialCommandGroup(
      new InstantCommand(() -> {
        this.resetOdometry(traj.getInitialPose());
        // TODO: I removed the isFirstPath parameter, as creating a new 
        // TODO: command will always reset the odometry anyway. Please 
        // TODO: let me know if you would like the functionality to be 
        // TODO: able to run multiple PathPlannerTrajectories in series, 
        // TODO: and multiple commands do not work. 
        // TODO: Please delete this after you have tested it.
      }),
      new PPRamseteCommand(
            traj,
            this::getPose, // Pose supplier
            new RamseteController(),
            new SimpleMotorFeedforward(Constants.MotionParameters.Drivetrain.k_s,
                Constants.MotionParameters.Drivetrain.k_v, Constants.MotionParameters.Drivetrain.k_a),
            drivetrain.m_kinematics, // DifferentialDriveKinematics
            this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
            new PIDController(Constants.MotionParameters.Drivetrain.k_p, Constants.MotionParameters.Drivetrain.k_i,
                Constants.MotionParameters.Drivetrain.k_d), // Left controller. Tune these values for your robot.
                                                            // Leaving them 0 will only
            // use feedforwards.
            new PIDController(Constants.MotionParameters.Drivetrain.k_p, Constants.MotionParameters.Drivetrain.k_i,
                Constants.MotionParameters.Drivetrain.k_d), // Right controller (usually the same values as left
                                                            // controller)
            this::outputVolts, // Voltage biconsumer
            true, // Should the path be automatically mirrored depending on alliance color.
                  // Optional, defaults to true
            drivetrain // Requires this drive subsystem
        ));

    this.addRequirements(drivetrain);
  }

  /**
   * Reset the robot's position and orientation stored in the odometry.
   * @param pose The position and orientation to set
   */
  public void resetOdometry(Pose2d pose) {
    this.m_drivetrain.resetEncoders();
    m_odometry.resetPosition(getGyroHeading(), m_drivetrain.getLeftEncoderDistance(), m_drivetrain.getRightEncoderDistance(), pose);
  }

  /**
   * Get the robot's position and orientation relative to the pitch.
   * @return The robot's pose relative to the pitch
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Get a WheelSpeeds object of the encoders' detected speeds, m/s.
   * @return A DifferentialDriveWheelSpeeds object from the encoders, m/s
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_drivetrain.getLeftEncoderRate(), m_drivetrain.getRightEncoderRate());
  }

  /**
   * Output the voltages provided to the drivetrain, with some extra debug info for autonomous driving.
   * @param vLeft Left voltage, V
   * @param vRight Right voltage, V
   */
  public void outputVolts(double vLeft, double vRight) {
    SmartDashboard.putNumber("FollowPath/Left Vel", vLeft);
    SmartDashboard.putNumber("FollowPath/Right Vel", vRight);
    SmartDashboard.putNumber("FollowPath/Left Pos", m_drivetrain.getLeftEncoderDistance());
    SmartDashboard.putNumber("FollowPath/Right Pos", m_drivetrain.getRightEncoderDistance());
    SmartDashboard.putString("FollowPath/Wheel Speeds", this.getWheelSpeeds().toString());

    m_drivetrain.outputVolts(vLeft, vRight);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Update the pose
    m_odometry.update(getGyroHeading(),
                      m_drivetrain.getLeftEncoderDistance(), m_drivetrain.getRightEncoderDistance());

    sequentialCommandGroup.execute();
  }
  
  private Rotation2d getGyroHeading(){
    return m_gyro.getAngle(IMUAxis.kZ); // TODO: Check axis
  }

  /* Events */

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sequentialCommandGroup.initialize();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sequentialCommandGroup.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return sequentialCommandGroup.isFinished();
  }
}
