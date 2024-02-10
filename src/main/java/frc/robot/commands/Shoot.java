// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swinger;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Shoot extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Swerve m_Swerve;
  private final Shooter m_Shooter;
  private final Intake m_Intake;
  private final Swinger m_Swinger;
  private final Limelight m_Limelight;
  private final int m_goalType;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Shoot(Swerve swerve, Shooter shooter, Intake intake, Swinger swinger, Limelight limelight, int goalType) {
    m_Swerve = swerve;
    m_Shooter = shooter;
    m_Intake = intake;
    m_Swinger = swinger;
    m_Limelight = limelight;
    m_goalType = goalType;
    addRequirements(swerve, shooter, intake, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new ManeuverOn(m_Swerve, m_Limelight, m_goalType)
      .andThen(new SwingToPosition(m_Swinger, m_goalType))
      .andThen(new DoShoot(m_Intake, m_Shooter, m_goalType));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
