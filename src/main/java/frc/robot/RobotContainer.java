// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Ports;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DifferentialDriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // NetworkTables subscribers (read)/publishers (write)
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable fmsInfo = inst.getTable("FMSInfo");
  BooleanSubscriber isRedAllianceSubscriber = fmsInfo.getBooleanTopic("IsRedAlliance").subscribe(false);


  // The robot's subsystems and commands are defined here...
  protected final DifferentialDriveSubsystem m_drivetrain = new DifferentialDriveSubsystem(isRedAllianceSubscriber);

  // Replace with CommandPS4Controller or CommandXboxController if needed
  private final CommandJoystick m_driverController = new CommandJoystick(Ports.kControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    Joystick j = m_driverController.getHID();
    Trigger action1Trigger = new JoystickButton(j, 8);
    Trigger action2Trigger = new JoystickButton(j, 7);
    
    action1Trigger.onTrue(Commands.runOnce(() -> {
      System.out.println("Action 1");
    }));
    
    action2Trigger.onTrue(Commands.runOnce(() -> {
      System.out.println("Action 2");
    }));

    // If the drivetrain is not running other commands, run arcade drive
    m_drivetrain.setDefaultCommand(Commands.run(() -> {
      double speed = -m_driverController.getY();
      double turn = m_driverController.getX();
      SmartDashboard.putNumber("Speed", speed);
      SmartDashboard.putNumber("Turn", turn);
      m_drivetrain.arcadeDrive(speed, turn);
      
    }, m_drivetrain));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public CommandBase getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto();
  }
}
