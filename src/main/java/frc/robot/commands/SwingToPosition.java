package frc.robot.commands;

import frc.robot.Constants.GoalTypeConstants;
import frc.robot.subsystems.Swinger;
import edu.wpi.first.wpilibj2.command.Command;

public class SwingToPosition extends Command {
  private final Swinger m_Swinger;
  private final int m_goalType;

  public SwingToPosition(Swinger swinger, int goalType) {
    m_Swinger = swinger;
    m_goalType = goalType;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swinger);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_goalType) {
      case GoalTypeConstants.AMP:
        break;
      case GoalTypeConstants.SPEAKER:
        break;
      case GoalTypeConstants.SOURCE_1:
        break;
      case GoalTypeConstants.SOURCE_2:
        break;
      case GoalTypeConstants.SOURCE_3:
        break;
      case GoalTypeConstants.TRAP:
        break;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
