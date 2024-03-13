package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swinger;
import edu.wpi.first.wpilibj2.command.Command;

public class ComboIntake extends Command {
  private final IntakeSubsystem m_Intake;
  private final Shooter m_Shooter;
  private final Swerve m_Swerve;
  private final Swinger m_Swinger;
  private final int m_GoalType;

  public ComboIntake(Swerve swerve, IntakeSubsystem intake, Shooter shooter, Swinger swinger, int goalType) {
    m_Swerve = swerve;
    m_Intake = intake;
    m_Shooter = shooter;
    m_Swinger = swinger;
    m_GoalType = goalType;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve, intake, shooter, swinger);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new SwingToPosition(m_Swinger, m_GoalType)
      .andThen(new ManeuverOn(m_Swerve, m_GoalType))
      .andThen(new DoIntake(m_Intake, m_Shooter));
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
