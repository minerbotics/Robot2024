package frc.robot.commands;

import frc.robot.Constants.GoalTypeConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class DoShoot extends Command {
  private final IntakeSubsystem m_intake;
  private final Shooter m_shooter;
  private final int m_goalType; 

  public DoShoot(IntakeSubsystem intake, Shooter shooter, int goalType) {
    m_intake = intake;
    m_shooter = shooter;
    m_goalType = goalType;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, shooter);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(m_goalType){
      case GoalTypeConstants.AMP:
        m_shooter.ampOut();
        m_intake.out();
        break;
      case GoalTypeConstants.SPEAKER:
        m_shooter.speakerOut();
        break;
      case GoalTypeConstants.TRAP:
        m_shooter.ampOut();
        m_intake.out();
        break;
      default:
        break;
    }
    

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
    m_shooter.stop();
  }
}
