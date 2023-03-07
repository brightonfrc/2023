// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

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
    public static final boolean k_DrivetrainStartInverted = false;
  }

  public static class Ports {
    // Controllers / joysticks
    public static final int k_controllerPort = 0;

    // Motor controllers
    public static final int k_DrivetrainMotorControllerPortL1 = 20;
    public static final int k_DrivetrainMotorControllerPortL2 = 2;
    public static final int k_DrivetrainMotorControllerPortR1 = 21;
    public static final int k_DrivetrainMotorControllerPortR2 = 10;
  }

  public static class MotionParameters {
    public static class Autobalance {
      public static final double k_p = 1;
      public static final double k_i = 1;
      public static final double k_d = 1;
      
      public static final double k_maxVelocity = 1;
      public static final double k_maxAcceleration = 1;
    }
    public static class Drivetrain {
      public static final double k_p = 1;
      public static final double k_i = 1;
      public static final double k_d = 1;
      
      public static final double k_s = 1;
      public static final double k_v = 1;
      public static final double k_a = 1;

      public static final double k_encoderDistancePerPulse = 0.01; // TODO
    }
  }
    

  public static class Measurements {
    public static class Drivetrain {
      public static final double k_wheelRadius = 0.075; // m
      public static final int k_encoderResolution = 2048;
      public static final int k_motorsPerCIMGearbox = 2;

      public static final double k_gearRatio = 10.75; // :1
      public static final double k_trackWidth = 0.55; // m
    }

    public static final double k_momentOfInertia = 5.0; // kg m^2
    public static final double k_mass = 5.0;

    public static final Transform3d k_robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));  // TODO - e.g. - Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  }

  public static class Strategy {
    public static final Pose2d k_startRed = new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d());
    public static final Pose2d k_startBlue = new Pose2d(new Translation2d(5.0, 0.0), new Rotation2d());
  }
}
