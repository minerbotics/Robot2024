// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.GoalTypeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoAmpScore;
import frc.robot.commands.AutoDriveForward;
import frc.robot.commands.AutoSpeakerScore;
import frc.robot.commands.ClimberDown;
import frc.robot.commands.ClimberUp;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DoIntake;
import frc.robot.commands.DoShoot;
import frc.robot.commands.ComboIntake;
import frc.robot.commands.ManeuverOn;
import frc.robot.commands.ManualSwing;
import frc.robot.commands.ComboShoot;
import frc.robot.commands.SwingToPosition;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swinger;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Subsystems
  private final Swerve m_Swerve;
  private final Climber m_Climber;
  private final IntakeSubsystem m_IntakeSubsystem;
  private final Shooter m_Shooter;
  private final Swinger m_Swinger;


  private final CommandXboxController m_driverController;
  private final CommandXboxController m_OperatorController;

  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Subsystems
    m_Swerve = new Swerve();
    m_Climber = new Climber();
    m_IntakeSubsystem = new IntakeSubsystem();
    m_Shooter = new Shooter();
    m_Swinger = new Swinger();

    // Controllers
    m_driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    m_OperatorController = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

    // Commands
//    NamedCommands.registerCommand("AmpShoot", new Shoot(m_Swerve, null, null, null, GoalTypeConstants.AMP));
    
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_Swerve.setDefaultCommand(new DefaultDriveCommand(
            m_Swerve,
            () -> -modifyAxis(m_driverController.getLeftY()) * Swerve.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_driverController.getLeftX()) * Swerve.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_driverController.getRightX()) * Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    m_Swinger.setDefaultCommand(new ManualSwing(m_Swinger, m_OperatorController));
    SmartDashboard.updateValues();
    // Configure the trigger bindings
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption("Score In Speaker", new AutoSpeakerScore(m_Swerve, m_Swinger, m_IntakeSubsystem, m_Shooter));
    autoChooser.addOption("Basic Drive Forward", new AutoDriveForward(m_Swerve));
    autoChooser.setDefaultOption("Score In Amp", new AutoAmpScore(m_Swerve, m_Swinger, m_IntakeSubsystem, m_Shooter));

    SmartDashboard.putData("Auto Chooser", autoChooser);

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
    
    m_driverController.start()
      .onTrue(new InstantCommand(() -> m_Swerve.zeroGyroscope()));

    /** DPAD commands for robot centric slow movement */
    m_driverController.povUp()
      .whileTrue(new DefaultDriveCommand(m_Swerve, () -> 0.5, () -> 0.0, () -> 0.0, true));
    m_driverController.povDown()
      .whileTrue(new DefaultDriveCommand(m_Swerve, () -> -0.5, () -> 0.0, () -> 0.0, true));
    m_driverController.povLeft()
      .whileTrue(new DefaultDriveCommand(m_Swerve, () -> 0.0, () -> 0.5, () -> 0.0, true));
    m_driverController.povRight()
      .whileTrue(new DefaultDriveCommand(m_Swerve, () -> 0.0, () -> -0.5, () -> 0.0, true));

    /** Climber commands */
    m_driverController.y()
      .onTrue(new ClimberUp(m_Climber));
    m_driverController.a()
      .onTrue(new ClimberDown(m_Climber));

    // Only 1 of the following 2 command bindings should be uncommented.
//    setOperatorComboCommandBindings();
    setOperatorManualCommandBindings();
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  /** Combo Commands (swing -> maneuver -> intake/shoot) */
  private void setOperatorComboCommandBindings() {
    m_OperatorController.x().whileTrue(new ComboShoot(m_Swerve, m_Shooter, m_IntakeSubsystem, m_Swinger, GoalTypeConstants.AMP));
    m_OperatorController.leftBumper().whileTrue(new ComboIntake(m_Swerve, m_IntakeSubsystem, m_Shooter, m_Swinger, GoalTypeConstants.SOURCE_1));
    m_OperatorController.leftBumper().and(m_OperatorController.rightBumper()).whileTrue(new ComboIntake(m_Swerve, m_IntakeSubsystem, m_Shooter, m_Swinger, GoalTypeConstants.SOURCE_2));
    m_OperatorController.rightBumper().whileTrue(new ComboIntake(m_Swerve, m_IntakeSubsystem, m_Shooter, m_Swinger, GoalTypeConstants.SOURCE_3));
    m_OperatorController.y().whileTrue(new ComboShoot(m_Swerve, m_Shooter, m_IntakeSubsystem, m_Swinger, GoalTypeConstants.SPEAKER));
  }

  /** Button Bindings for when PID isn't working and arm movement is manual. */
  private void setOperatorManualCommandBindings() {
    // Intake
    m_OperatorController.rightTrigger(0.5).whileTrue(new ManeuverOn(m_Swerve, GoalTypeConstants.SOURCE_1, false));
    m_OperatorController.leftTrigger(0.5).whileTrue(new ManeuverOn(m_Swerve, GoalTypeConstants.SOURCE_1, false));
    m_OperatorController.leftBumper().whileTrue(new DoIntake(m_IntakeSubsystem, m_Shooter));
    m_OperatorController.rightBumper().whileTrue(new DoIntake(m_IntakeSubsystem, m_Shooter));
    
    // AmpShoot
    m_OperatorController.a().whileTrue(new ManeuverOn(m_Swerve, GoalTypeConstants.AMP, false));
    m_OperatorController.x().whileTrue(new DoShoot(m_IntakeSubsystem, m_Shooter, GoalTypeConstants.AMP));

    // SpeakerShoot
    m_OperatorController.b().whileTrue(new ManeuverOn(m_Swerve, GoalTypeConstants.SPEAKER, false));
    m_OperatorController.y().whileTrue(new DoShoot(m_IntakeSubsystem, m_Shooter, GoalTypeConstants.SPEAKER));

    // Arm Positions
    m_OperatorController.povDown().onTrue(new SwingToPosition(m_Swinger, 0));
    m_OperatorController.povLeft().onTrue(new SwingToPosition(m_Swinger, GoalTypeConstants.SOURCE_1));
    m_OperatorController.povRight().onTrue(new SwingToPosition(m_Swinger, GoalTypeConstants.AMP));
    m_OperatorController.povUp().onTrue(new SwingToPosition(m_Swinger, GoalTypeConstants.SPEAKER));
  }

}
