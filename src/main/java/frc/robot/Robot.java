// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.photonvision.PhotonCamera;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.cv.AprilTagNavigator;
import frc.robot.dataStorageClasses.AutonomousSelection;
import frc.robot.dataStorageClasses.ModeSelection;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private AprilTagNavigator m_aprilTagNavigator;

  // NetworkTables subscribers (read)/publishers (write)
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  
  private SendableChooser<AutonomousSelection> m_autonomousChooser;
  private SendableChooser<Alliance> m_allianceChooser;
  private SendableChooser<ModeSelection> m_modeChooser;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Camera preview on Shuffleboard
    CameraServer.startAutomaticCapture();

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Allow the user to select the desired autonomous from smartdasboard
    m_autonomousChooser = new SendableChooser<AutonomousSelection>();
    m_autonomousChooser.setDefaultOption("AutoBalance Only FORWARDS [Fallback]", AutonomousSelection.AutoBalanceOnlyForwards);
    m_autonomousChooser.setDefaultOption("AutoBalance Only REVERSE [Fallback]", AutonomousSelection.AutoBalanceOnlyReverse);
    m_autonomousChooser.setDefaultOption("Closest Path + AutoBalance [Basic]", AutonomousSelection.ClosestPathAndAutoBalance);
    m_autonomousChooser.setDefaultOption("Middle Path + AutoBalance [Basic]", AutonomousSelection.MiddlePathAndAutoBalance);
    m_autonomousChooser.setDefaultOption("Furthest Path + AutoBalance [Basic]", AutonomousSelection.FurthestPathAndAutoBalance);
    // m_autonomousChooser.addOption("Score and balance", AutonomousSelection.ScoreAndBalance);
    SmartDashboard.putData("Choosers/Auto choices", m_autonomousChooser);
    
    m_allianceChooser = new SendableChooser<TeamColourSelection>();
    m_allianceChooser.setDefaultOption("Red", TeamColourSelection.Red);
    m_allianceChooser.addOption("Blue", TeamColourSelection.Blue);
    SmartDashboard.putData("Choosers/Team colour", m_allianceChooser);

    m_modeChooser = new SendableChooser<ModeSelection>();
    m_modeChooser.setDefaultOption("Game", ModeSelection.Game);
    m_modeChooser.addOption("Test SparkMax", ModeSelection.TestSparkMax);
    m_modeChooser.addOption("Test Talons", ModeSelection.TestTalon);
    SmartDashboard.putData("Choosers/Mode", m_modeChooser);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // Set up the subsystems before using them
    m_robotContainer.setupSubsystems(m_modeChooser.getSelected());

    // Find the auto command that was selected to be run
    m_autonomousCommand = m_robotContainer.getAutonomousCommand(m_autonomousChooser.getSelected(), m_allianceChooser.getSelected());

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    try {
      m_aprilTagNavigator = new AprilTagNavigator(new PhotonCamera(inst, "camera"));
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // PhotonTrackedTarget aprilTag =  m_aprilTagNavigator.getAprilTag();
    // if(aprilTag != null) {
    //   SmartDashboard.putNumber("AprilTag/ID", aprilTag.getFiducialId());
    //   SmartDashboard.putNumber("AprilTag/Yaw (+> -<)", aprilTag.getYaw());
    //   SmartDashboard.putNumber("AprilTag/Pitch (+^ -v)", aprilTag.getPitch());

    //   Optional<EstimatedRobotPose> pose = m_aprilTagNavigator.getRobotPose();
    //   if(!pose.isEmpty()) {
    //     SmartDashboard.putString("AprilTag/Last Robot Pose", pose.get().toString());
    //   }
    //   SmartDashboard.putString("AprilTag/Robot Pose", pose.toString());
    // } else {
    //   SmartDashboard.putNumber("AprilTag/ID", 0);
    //   SmartDashboard.putNumber("AprilTag/Yaw (+> -<)", 0);
    //   SmartDashboard.putNumber("AprilTag/Pitch (+^ -v)", 0);
    // }
  }

  @Override
  public void teleopInit() {
    // Set up the subsystems before using them
    m_robotContainer.setupSubsystems(m_modeChooser.getSelected());

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_robotContainer.logGyro();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    Simulation.updateSimulation();
  }
}
