// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int PIGEON_ID = 15;
  /**
   * The left-to-right distance between the drivetrain wheels
   *
   * Should be measured from center to center.
   */
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = .4572;
  /**
   * The front-to-back distance between the drivetrain wheels.
   *
   * Should be measured from center to center.
   */
  public static final double DRIVETRAIN_WHEELBASE_METERS = .600075;

  public static final double DRIVETRAIN_WHEELBASE_RADIUS = Math.sqrt(Math.pow(DRIVETRAIN_TRACKWIDTH_METERS, 2) + Math.pow(DRIVETRAIN_TRACKWIDTH_METERS, 2)) / 2;

  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 5;
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 6;
  public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 9;
  public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(306.3);

  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 1;
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 2;
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 11;
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(82.8);

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7;
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8;
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 10;
  public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(35.5);

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 3;
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 4;
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12;
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(141.6);

  public static final double TRANSLATION_SLEW = 4.5;
  public static final double ROTATION_SLEW = 10;

  public static final int PCM_ID = 0;

  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  public static class ShooterConstants {
    public static final int MID_SHOOTER_MOTOR = 20;
    public static final int BOTTOM_SHOOTER_MOTOR = 21;
    public static final int TOP_RIGHT_SHOOTER_MOTOR = 26;
    public static final int TOP_LEFT_SHOOTER_MOTOR = 22;
  }

  public static class IntakeConstants {
    public static final int INTAKE_MOTOR = 23;
  }

  public static class SwingConstants {
    public static final int DIO_PORT = 0;
    public static final int LEFT_SWING_MOTOR = 24;
    public static final int RIGHT_SWING_MOTOR = 25;
    public static final double DOWN_POSITION = 0.0;
    public static final double AMP_POSITION = -70.0;
    public static final double SPEAKER_POSITION = -35.0;
    public static final double OFFSET_SPEAKER_POSITION = -40.0;
    public static final double SOURCE_POSITION = -100.0;
    public static final double TRAP_POSITION = -90.0;
    public static final double SWING_POSITION_TOLERANCE = 2.0;
    public static final double SWING_VELOCITY_TOLERANCE = 2.0;
  }

  public static class ClimberConstants {
    public static final int CLIMBER_UP_CHANNEL = 4;    
    public static final int CLIMBER_DOWN_CHANNEL = 5;
  }

  public static class GoalTypeConstants {
    public static final int AMP = 1;
    public static final int SPEAKER = 2;
    public static final int SOURCE_1 = 3;
    public static final int SOURCE_2 = 4;
    public static final int SOURCE_3 = 5;
    public static final int TRAP = 6; 
    public static final int OFFSET_SPEAKER = 7;
  }

  public static class AutoConstants {
    public static final PIDConstants TRANSLATION_AUTO_PID = new PIDConstants(3, 0, 0);
    public static final PIDConstants ROTATION_AUTO_PID = new PIDConstants(3, 0, 0);
  }
  
}
