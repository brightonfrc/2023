// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Ports;
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
  private Intake m_intake;
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
      // No bindings, everything done from the smart dashboard
      // Just start the sparkmax test command
      new SparkMaxTester();
      
      return;
      
      // NOTE: Game is the default
      default:
        // Only instantiate the subsystems if we need them
        this.m_arm = new Arm();
        this.m_intake = new Intake();
        this.m_drivetrain = new DifferentialDriveWrapper();
    }
  }
  
  /** Sets up bindings to be used in a game */
  public void gameTeleopBindings(){
    Joystick j = m_driverController.getHID();

    // Triggers for intake
    Trigger grabTrigger = new JoystickButton(j, 8);
    Trigger releaseTrigger = new JoystickButton(j, 9);
    
    grabTrigger.onTrue(new IntakeGrab(m_intake));
    releaseTrigger.onTrue(new IntakeRelease(m_intake));
    
    // If the drivetrain is not running other commands, run arcade drive
    m_drivetrain.setDefaultCommand(Commands.run(() -> {
      double speed = -m_driverController.getRightY();
      double turn = -m_driverController.getRightX();
      SmartDashboard.putNumber("speed", speed);
      
      // Reverse the turning direction when going backwards, like a car
      // Only assume we are going backwards if we are outside the deadband
      // if (speed < RobotDriveBase.kDefaultDeadband) turn *= -1;

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
        return new FollowPath(m_drivetrain, m_gyro, PathPlanner.loadPath("DriveForward", new PathConstraints(1, 0.25)));
    }
  }

  public void logGyro() {
    // TODO: atm always returns same axis
    SmartDashboard.putNumber("Gyro/X", m_gyro.getAngle(IMUAxis.kX).getDegrees());
    // SmartDashboard.putNumber("Gyro/Y", m_gyro.getAngle(IMUAxis.kY).getDegrees());
    // SmartDashboard.putNumber("Gyro/Z", m_gyro.getAngle(IMUAxis.kZ).getDegrees());
  }
}
