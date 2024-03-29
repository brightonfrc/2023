// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Ports;
import frc.robot.commands.ArmManualLevel;
import frc.robot.commands.ArmSetLevel;
import frc.robot.commands.AutoBalanceV2;
import frc.robot.commands.DriveForwardsTime;
import frc.robot.commands.FollowPath;
import frc.robot.commands.IntakeGrab;
import frc.robot.commands.IntakeRelease;
import frc.robot.commands.TurnOnSpot;
import frc.robot.commands.TestCommands.TestDrivetrainPID;
import frc.robot.dataStorageClasses.AutoCubeScoringStrategy;
import frc.robot.dataStorageClasses.AutoMotionScoringStrategy;
import frc.robot.dataStorageClasses.ModeSelection;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DifferentialDriveWrapper;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
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
  private Intake m_intake;
  private Arm m_arm;
  private LED m_intakeLED;

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
      case TestSpeedPIDDrive:
        this.m_drivetrain = new DifferentialDriveWrapper();
        this.m_drivetrain.setDefaultCommand(new TestDrivetrainPID(m_drivetrain));
        return;
      // NOTE: Game is the default
      default:
        // Only instantiate the subsystems if we need them
        this.m_drivetrain = new DifferentialDriveWrapper();
        this.m_intake = new Intake();
        this.m_arm = new Arm(1);
        this.m_intakeLED = new LED(Constants.Ports.k_LEDPort);
        
        // Configure all the bindings and default commands
        gameTeleopBindings();
    }
  }
  
  /** Sets up bindings to be used in a game */
  public void gameTeleopBindings(){
    XboxController controller = m_driverController.getHID();

    // Bumpers for intake
    Trigger grabTrigger = m_driverController.leftBumper();
    Trigger releaseTrigger = m_driverController.rightBumper();
    Trigger armGroundTrigger = m_driverController.x();
    Trigger armIntakeTrigger = m_driverController.a();
    Trigger armMidTrigger = m_driverController.b();
    Trigger armHighTrigger = m_driverController.y();
    
    grabTrigger.onTrue(new IntakeGrab(m_intake, this.m_intakeLED));
    releaseTrigger.onTrue(new IntakeRelease(m_intake));
    armGroundTrigger.onTrue(new ArmSetLevel(m_arm, 0));
    armIntakeTrigger.onTrue(new ArmSetLevel(m_arm, 1));
    armMidTrigger.onTrue(new ArmSetLevel(m_arm, 2));
    armHighTrigger.onTrue(new ArmSetLevel(m_arm, 3));
    
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
  public CommandBase getAutonomousCommand(AutoCubeScoringStrategy cubeStrat, AutoMotionScoringStrategy motionStrat, Alliance alliance) {
    
    ArrayList<Command> commands = new ArrayList<Command>();
    
    // Notice the defaults, default is to push and balance

    if (cubeStrat == AutoCubeScoringStrategy.None) {}
    else if (cubeStrat == AutoCubeScoringStrategy.ShootMid) {
      // start the intake
      commands.add(new IntakeGrab(m_intake, this.m_intakeLED));
      // Wait a bit for it to take hold of the cube
      commands.add(new WaitCommand(0.1));
      // Start moving the arm to the high preset
      commands.add(new ArmSetLevel(m_arm, 2));
      // Wait for the arm to reach it
      commands.add(new WaitUntilCommand(m_arm::isArmSettled));
      // Shoot it
      commands.add(new IntakeRelease(m_intake));
    } else {
      // push
      commands.add(new DriveForwardsTime(m_drivetrain, 500, 0.3));
      commands.add(new DriveForwardsTime(m_drivetrain, 600, -0.3));
    }

    if (motionStrat == AutoMotionScoringStrategy.None) {}
    else if (motionStrat == AutoMotionScoringStrategy.HundredEightyDegTurn) {
      commands.add(new TurnOnSpot(m_drivetrain, m_gyro, 180));
    }
    else if (motionStrat == AutoMotionScoringStrategy.ClosestBalance){
      commands.add(new FollowPath(m_drivetrain, m_gyro, "2Closest", alliance));
      commands.add(new AutoBalanceV2(m_gyro, m_drivetrain, false));
    } else {
      // autobalance
      commands.add(new AutoBalanceV2(m_gyro, m_drivetrain, false));
    }
    
    // Do not do anything if it is empty
    if (commands.isEmpty()) return null;
    // Otherwise, trigger them one by one
    // This passes the array of commands to the command group
    return new SequentialCommandGroup(commands.toArray(Command[]::new));
  }
}
