// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int PIGEON_ID = 20;
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

  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 5;
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 6;
  public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 9;
  public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(142.99);

  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 1;
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 2;
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 11;
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(219.81);

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7;
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8;
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 10;
  public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(265.43);

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 3;
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 4;
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12;
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(100.46);

  public static final double TRANSLATION_SLEW = 1.55;
  public static final double ROTATION_SLEW = 3.00;

  public static final int PCM_ID = 0;

  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  public static class ShooterConstants {
    public static final int TOP_SHOOTER_MOTOR = 20;
    public static final int MID_SHOOTER_MOTOR = 21;
    public static final int BOTTOM_SHOOTER_MOTOR = 22;
  }

  public static class IntakeConstants {
    public static final int INTAKE_MOTOR = 23;
  }

  public static class SwingConstants {
    public static final int LEFT_SWING_MOTOR = 24;
    public static final int RIGHT_SWING_MOTOR = 25;
    public static final double AMP_POSITION = 35;
    public static final double SPEAKER_POSITION = -35;
    public static final double SOURCE_POSITION = 90;
    public static final double TRAP_POSITION = 90;
    public static final double SWING_POSITION_TOLERANCE = 3;
  }

  public static class ClimberConstants {
    public static final int LEFT_CLIMB_UP_CHANNEL = 1;    
    public static final int LEFT_CLIMB_DOWN_CHANNEL = 2;
    public static final int RIGHT_CLIMB_UP_CHANNEL = 3;
    public static final int RIGHT_CLIMB_DOWN_CHANNEL = 4;

  }

  public static class GoalTypeConstants {
    public static final int AMP = 1;
    public static final int SPEAKER = 2;
    public static final int SOURCE_1 = 3;
    public static final int SOURCE_2 = 4;
    public static final int SOURCE_3 = 5;
    public static final int TRAP = 6; 
  }
  
}
