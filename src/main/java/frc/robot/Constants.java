// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.util.Units;
import frc.robot.dataStorageClasses.ArmPositionCounts;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class RobotSettings {
    public static final double k_turntableMaxPower = 0.4;
    public static final boolean k_drivetrainStartInverted = false;
    public static final double k_normalTurnSensitivity = 0.5;
    public static final double k_slowedTurnSensitivity = 0.5;
    public static final double k_normalSpeedSensitivity = 1.0;
    public static final double k_slowedSpeedSensitivity = 0.5;
  }

  public static class Ports {
    // Controllers / joysticks
    public static final int k_controllerPort = 0;

    // Motor controllers
    // These numbers correspond to PID ids
    // Talons are on ids in 20s, sparkMaxes are on ids in 30s
    public static final int k_drivetrainMotorControllerPortL1 = 1;
    public static final int k_drivetrainMotorControllerPortR1 = 2;
    public static final int k_drivetrainMotorControllerPortL2 = 3;
    public static final int k_drivetrainMotorControllerPortR2 = 4;
    
    public static final int k_encoderPortAL = 0;
    public static final int k_encoderPortBL = 1;
    public static final int k_encoderPortAR = 2;
    public static final int k_encoderPortBR = 3;

    public static final int k_armChainMotor = 31;
    public static final int k_armCableMotor = 30;

    public static int k_intakeMotor = 20;
    public static final int k_turntableMotor = 21;
  }
  
  public static class MotionParameters {
    public static class AutoBalance {
      /**
       * Robot speed when driving up slope of charge station
       */
      public static final double k_robotSpeedSlow = 0.2;
      /**
       * The speed used to correct the small error when very close to the setpoint
       */
      public static final double k_robotSpeedCorrection = 0.;
      /**
       * Robot speed when driving towards charge station
       */
      public static final double k_robotSpeedFast = 0.4;
      /**
       * Minimum Angle at which the robot is driving up the charge station (degrees)
       */
      public static final double k_onChargeStationDegree = 13.0;
      /**
       * Maximum Angle at which the robot is level, on top of the charge station (degrees)
       */
      public static final double k_levelDegree = 2.0; // In manual, 2.5deg
      /**
       * Time delay from start of state trigger to state entry (seconds)
       */
      public static final double k_debounceTime = 0.2;
    }

    public static class AutoBalanceReversed {
      /**
       * Robot speed when driving up slope of charge station
       */
      public static final double k_robotSpeedSlow = 0.2;
      /**
       * The speed used to correct the small error when very close to the setpoint
       */
      public static final double k_robotSpeedCorrection = 0.1;
      /**
       * Robot speed when driving towards charge station
       */
      public static final double k_robotSpeedFast = 0.4;
      /**
       * Minimum Angle at which the robot is driving up the charge station (degrees)
       */
      public static final double k_onChargeStationDegree = 13.0;
      /**
       * Maximum Angle at which the robot is level, on top of the charge station (degrees)
       */
      public static final double k_levelDegree = 2.0; // In manual, 2.5deg
      /**
       * Time delay from start of state trigger to state entry (seconds)
       */
      public static final double k_debounceTime = 0.2;
    }
    
    public static class Drivetrain {
      public static final double k_p = 4.3831;
      public static final double k_i = 0;
      public static final double k_d = 0;

      public static final double k_speedThresholdForTurnInPlace = 0.4;
      
      // Determine these using sysid: https://docs.wpilib.org/en/stable/docs/software/pathplanning/system-identification/configuring-project.html 
      public static final double k_s = 0.59581;
      public static final double k_v = 3.1502;
      public static final double k_a = 0.80703;

      public static final double k_distancePerEncoderPulse = Math.PI * Constants.Measurements.Drivetrain.k_wheelDiameter / Constants.Measurements.Drivetrain.k_encoderPulsesPerRotation; // (distance per pulse = (circumference = pi * r^2) / pulses per rotation)
    }
  }
    
  public static class Arm {
    // chain motor, then cable motor
    public static final ArmPositionCounts[] k_armMotorPositionCounts = {
      new ArmPositionCounts(0, 0, "Stowed"),
      new ArmPositionCounts(0, -180, "Ground"),
      new ArmPositionCounts(-26, -250, "Mid"),
      new ArmPositionCounts(-33, -180, "Top")
    };
    
    public static final double cableMotorP = 0.00008;
    public static final double cableMotorI = 1e-6;
    public static final double cableMotorD = 0;
    public static final double cableMotorFF = 0.0003;
    
  }
 
  public static class Measurements {
    public static class Drivetrain {
      public static final double k_wheelDiameter = Units.inchesToMeters(6); // m
      public static final int k_encoderPulsesPerRotation = 2048;

      public static final double k_trackWidth = 0.66981; // m
    }

    public static class Turntable {
      public static final int k_encoderPulsesPerRotation = 4096;
    }
  }
}
