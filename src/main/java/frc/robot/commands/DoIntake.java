package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class DoIntake extends Command {
  private final IntakeSubsystem m_Intake;
  private final Shooter m_Shooter;

  public DoIntake(IntakeSubsystem intake, Shooter shooter) {
    m_Intake = intake;
    m_Shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, shooter);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Intake.in();
    m_Shooter.intakeIn();
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
