// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.dataStorageClasses.AutoCubeScoringStrategy;
import frc.robot.dataStorageClasses.AutoMotionScoringStrategy;
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

  // NetworkTables subscribers (read)/publishers (write)
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  
  private SendableChooser<AutoCubeScoringStrategy> m_autoCubeStratChooser;
  private SendableChooser<AutoMotionScoringStrategy> m_autoMotionStratChooser;
  private SendableChooser<ModeSelection> m_teleopModeChooser;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Camera preview on Shuffleboard
    // CameraServer.startAutomaticCapture();

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Allow the user to select the desired autonomous from smartdasboard
    // m_autonomousChooser = new SendableChooser<AutonomousSelection>();
    // m_autonomousChooser.setDefaultOption("Push + AutoBalance [Fallback]", AutonomousSelection.PushThenAutoBalance);
    // m_autonomousChooser.addOption("Push Only [Fallback]", AutonomousSelection.PushOnly);
    // m_autonomousChooser.addOption("AutoBalance Only [Fallback]", AutonomousSelection.AutoBalanceOnly);
    // m_autonomousChooser.addOption("Closest Path + AutoBalance [Basic]", AutonomousSelection.ClosestPathAndAutoBalance);
    // m_autonomousChooser.addOption("Middle Path + AutoBalance [Basic]", AutonomousSelection.MiddlePathAndAutoBalance);
    // m_autonomousChooser.addOption("Furthest Path + AutoBalance [Basic]", AutonomousSelection.FurthestPathAndAutoBalance);
    // m_autonomousChooser.addOption("Closest Exit Community + AutoBalance [Average]", AutonomousSelection.ClosestExitCommunityAndAutoBalance);
    // m_autonomousChooser.addOption("Furthest Exit Community + AutoBalance [Average]", AutonomousSelection.FurthestExitCommunityAndAutoBalance);
    // SmartDashboard.putData("Choosers/Auto choices", m_autonomousChooser);
    m_autoCubeStratChooser = new SendableChooser<AutoCubeScoringStrategy>();
    m_autoCubeStratChooser.setDefaultOption("Push cube", AutoCubeScoringStrategy.Push);
    m_autoCubeStratChooser.addOption("None", AutoCubeScoringStrategy.None);
    m_autoCubeStratChooser.addOption("ShootMid", AutoCubeScoringStrategy.ShootMid);
    SmartDashboard.putData("Choosers/Cube strat", m_autoCubeStratChooser);

    m_autoMotionStratChooser = new SendableChooser<AutoMotionScoringStrategy>();
    m_autoMotionStratChooser.setDefaultOption("Balance in front", AutoMotionScoringStrategy.AutoBalance);
    m_autoMotionStratChooser.addOption("Leave and balance Closest", AutoMotionScoringStrategy.ClosestBalance);
    m_autoMotionStratChooser.addOption("180deg turn", AutoMotionScoringStrategy.HundredEightyDegTurn
    );
    m_autoMotionStratChooser.addOption("None", AutoMotionScoringStrategy.None);
    SmartDashboard.putData("Choosers/Motion strat", m_autoMotionStratChooser);

    m_teleopModeChooser = new SendableChooser<ModeSelection>();
    m_teleopModeChooser.setDefaultOption("Game", ModeSelection.Game);
    m_teleopModeChooser.addOption("Test SparkMax", ModeSelection.TestSparkMax);
    m_teleopModeChooser.addOption("Test drive speed PID", ModeSelection.TestSpeedPIDDrive);
    SmartDashboard.putData("Choosers/Mode", m_teleopModeChooser);
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
    m_robotContainer.setupSubsystems(m_teleopModeChooser.getSelected());
    
    Alliance currentAlliance = DriverStation.getAlliance();

    // Find the auto command that was selected to be run
    m_autonomousCommand = m_robotContainer.getAutonomousCommand(m_autoCubeStratChooser.getSelected(), m_autoMotionStratChooser.getSelected(), currentAlliance);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // Set up the subsystems before using them
    m_robotContainer.setupSubsystems(m_teleopModeChooser.getSelected());

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
  public void teleopPeriodic() {}

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
