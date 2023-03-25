// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Ports;
import frc.robot.commands.ArmManualLevel;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.FollowPath;
import frc.robot.commands.IntakeGrab;
import frc.robot.commands.IntakeRelease;
import frc.robot.dataStorageClasses.AutonomousSelection;
import frc.robot.dataStorageClasses.ModeSelection;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DifferentialDriveWrapper;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Intake;
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

  private boolean areSubsystemsSetUp = false;

  // Replace with CommandPS4Controller or CommandXboxController if needed
  private final CommandXboxController m_driverController = new CommandXboxController(Ports.k_controllerPort);

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
        // No bindings, everything done from the smart dashboard or from inside subsystems
        new SparkMaxTester();
        return;
      // NOTE: Game is the default
      default:
        // Only instantiate the subsystems if we need them
        this.m_drivetrain = new DifferentialDriveWrapper();
        this.m_arm = new Arm();
        
        // Configure all the bindings and default commands
        gameTeleopBindings();
    }
  }
  
  /** Sets up bindings to be used in a game */
  public void gameTeleopBindings(){
    XboxController controller = m_driverController.getHID();

    // Bumpers for intake
    // Trigger grabTrigger = m_driverController.leftBumper();
    // Trigger releaseTrigger = m_driverController.rightBumper();
    
    // grabTrigger.onTrue(new IntakeGrab(m_intake));
    // releaseTrigger.onTrue(new IntakeRelease(m_intake));
    
    m_arm.setDefaultCommand(new ArmManualLevel(m_arm));

    // If the drivetrain is not running other commands, run arcade drive with right joystick
    m_drivetrain.setDefaultCommand(Commands.run(() -> {
      double speed = -m_driverController.getRightY();
      double turn = -m_driverController.getRightX();

      if(controller.getRightStickButton()) {
        // Slow down
        speed *= Constants.RobotSettings.k_slowedSpeedSensitivity;
        turn *= Constants.RobotSettings.k_slowedTurnSensitivity;
      } else {
        speed *= Constants.RobotSettings.k_normalSpeedSensitivity;
        turn *= Constants.RobotSettings.k_normalTurnSensitivity;
      }
      
      // Reverse the turning direction when going backwards, like a car
      // Only assume we are going backwards if we are outside the deadband
      // if (speed < RobotDriveBase.kDefaultDeadband) turn *= -1;

      m_drivetrain.drive(speed, turn);
    }, m_drivetrain));
  }

  /**
   * Use this to get autonomous commands for the subsystems
   *
   * @return the command to run in autonomous
   */
  public CommandBase getAutonomousCommand(AutonomousSelection commandSelection, Alliance alliance) {
    
    var autobalanceCommand = new AutoBalance(m_gyro, m_drivetrain, false);

    switch (commandSelection) {
      case AutoBalanceOnly:
        return autobalanceCommand;
      case ClosestPathAndAutoBalance:
        return new SequentialCommandGroup(new FollowPath(m_drivetrain, m_gyro, "1Closest", alliance), autobalanceCommand);
      case MiddlePathAndAutoBalance:
        return new SequentialCommandGroup(new FollowPath(m_drivetrain, m_gyro, "1Middle", alliance), autobalanceCommand);
      case FurthestPathAndAutoBalance:
        return new SequentialCommandGroup(new FollowPath(m_drivetrain, m_gyro, "1Furthest", alliance), autobalanceCommand);
      default:
        return new InstantCommand(() -> {
          System.out.println("This autonomous strategy was not configured.");
        });
    }
  }
}
