// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Ports;
import frc.robot.commands.AutoBalance;
import frc.robot.dataStorageClasses.AutonomousSelection;
import frc.robot.dataStorageClasses.ModeSelection;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DifferentialDriveWrapper;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.testSubsystems.SparkMaxTester;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Gyro m_gyro = new Gyro();
  // The robot's subsystems and commands are defined here...
  private DifferentialDriveWrapper m_drivetrain;
  private Arm m_arm;
  private SparkMaxTester m_sparkMaxTester;

  private boolean areSubsystemsSetUp = false;

  // Replace with CommandPS4Controller or CommandXboxController if needed
  private final CommandJoystick m_driverController = new CommandJoystick(Ports.k_controllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {}

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  public void setupSubsystems(ModeSelection mode) {
    // Do not reinitialise the subsystems
    if (areSubsystemsSetUp) return;
    areSubsystemsSetUp = true;
    // Note: We are also creating the required subsystems for non-game modes
    switch (mode) {
      case TestSparkMax:
      // No bindings, everything done from the smart dashboard
      // Just start the sparkmax test command
      m_sparkMaxTester = new SparkMaxTester();
      
      return;
      
      // NOTE: Game is the default
      default:
        // Only instantiate the subsystems if we need them
        // this.m_arm = new Arm();
        this.m_drivetrain = new DifferentialDriveWrapper(m_gyro);
    }
  }
  
  /** Sets up bindings to be used in a game */
  public void gameTeleopBindings(){
    Joystick j = m_driverController.getHID();
    Trigger action1Trigger = new JoystickButton(j, 8);
    
    action1Trigger.onTrue(new AutoBalance(m_gyro, m_drivetrain));
    
    // If the drivetrain is not running other commands, run arcade drive
    m_drivetrain.setDefaultCommand(Commands.run(() -> {
      // double speed = SmartDashboard.getNumber("Speed", 0);
      // double turn = SmartDashboard.getNumber("Turn", 0);

      double speed = -m_driverController.getY();
      double turn = m_driverController.getX();
      SmartDashboard.putNumber("Speed", speed);
      SmartDashboard.putNumber("Turn", turn);
      m_drivetrain.drive(speed, turn);
    }, m_drivetrain));
  }

  /**
   * Use this to get autonomous commands for the subsystems
   *
   * @return the command to run in autonomous
   */
  public CommandBase getAutonomousCommand(AutonomousSelection commandSelection) {
    switch (commandSelection) {
      default:
        return m_drivetrain.followTrajectoryCommand(PathPlanner.loadPath("Test", new PathConstraints(1, 0.25), true), true);
    }
  }
}
