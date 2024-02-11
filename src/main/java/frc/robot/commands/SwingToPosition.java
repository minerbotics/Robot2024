package frc.robot.commands;

import frc.robot.Constants.GoalTypeConstants;
import frc.robot.Constants.SwingConstants;
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
    double position = 0;
    switch (m_goalType) {
      case GoalTypeConstants.AMP:
        position = SwingConstants.AMP_POSITION;
        break;
      case GoalTypeConstants.SPEAKER:
        position = SwingConstants.SPEAKER_POSITION;
        break;
      case GoalTypeConstants.SOURCE_1:
      case GoalTypeConstants.SOURCE_2:
      case GoalTypeConstants.SOURCE_3:
        position = SwingConstants.SOURCE_POSITION;
        break;
      case GoalTypeConstants.TRAP:
        position = SwingConstants.TRAP_POSITION;
        break;
      default:
        position = 0;
        break;
    }
    m_Swinger.swingToPosition(position);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return inPosition(m_goalType);
  }

  public boolean inPosition(int goalType) {
    boolean inPosition = false;
    switch (goalType) {
      case GoalTypeConstants.AMP:
        if (m_Swinger.getPosition() == SwingConstants.AMP_POSITION) {
          inPosition = true;
        }
        break;
      case GoalTypeConstants.SOURCE_1:
      case GoalTypeConstants.SOURCE_2:
      case GoalTypeConstants.SOURCE_3:
        if (m_Swinger.getPosition() == SwingConstants.SOURCE_POSITION) {
          inPosition = true;
        }
        break;
      case GoalTypeConstants.SPEAKER:
        if (m_Swinger.getPosition() == SwingConstants.SPEAKER_POSITION) {
          inPosition = true;
        }
        break;
      case GoalTypeConstants.TRAP:
        if (m_Swinger.getPosition() == SwingConstants.TRAP_POSITION) {
          inPosition = true;
        }
        break;
      default:
        inPosition = false;
        break;
    }
    return inPosition;
  }
}
