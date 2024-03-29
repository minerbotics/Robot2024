package frc.robot.commands;

import frc.robot.Constants.GoalTypeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swinger;
import edu.wpi.first.wpilibj2.command.Command;

public class ComboShoot extends Command {
  private final Swerve m_Swerve;
  private final Shooter m_Shooter;
  private final IntakeSubsystem m_Intake;
  private final Swinger m_Swinger;
  private final int m_goalType;

  public ComboShoot(Swerve swerve, Shooter shooter, IntakeSubsystem intake, Swinger swinger, int goalType) {
    m_Swerve = swerve;
    m_Shooter = shooter;
    m_Intake = intake;
    m_Swinger = swinger;
    m_goalType = goalType;
    addRequirements(swerve, shooter, intake, swinger);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_goalType != GoalTypeConstants.TRAP) {
      new SwingToPosition(m_Swinger, m_goalType)
        .andThen(new ManeuverOn(m_Swerve, m_goalType))
        .andThen(new DoShoot(m_Intake, m_Shooter, m_goalType));
    } else {
      new SwingToPosition(m_Swinger, m_goalType)
        .andThen(new DoShoot(m_Intake, m_Shooter, m_goalType));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Swinger.swingToPosition(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
