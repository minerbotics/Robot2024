package frc.robot.commands;

import frc.robot.Constants.GoalTypeConstants;
import frc.robot.Constants.SwingConstants;
import frc.robot.subsystems.Swinger;
import edu.wpi.first.wpilibj2.command.Command;

public class SwingToPosition extends Command {
  private final Swinger m_Swinger;
  private final int m_goalType;
  private double targetPosition;

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
        this.targetPosition = SwingConstants.AMP_POSITION;
        break;
      case GoalTypeConstants.SPEAKER:
        this.targetPosition = SwingConstants.SPEAKER_POSITION;
        break;
      case GoalTypeConstants.SOURCE_1:
      case GoalTypeConstants.SOURCE_2:
      case GoalTypeConstants.SOURCE_3:
        this.targetPosition = SwingConstants.SOURCE_POSITION;
        break;
      case GoalTypeConstants.TRAP:
        this.targetPosition = SwingConstants.TRAP_POSITION;
        break;
      default:
        this.targetPosition = 0;
        break;
    }
    m_Swinger.swingToPosition(this.targetPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_Swinger.getPosition() - this.targetPosition) <= SwingConstants.SWING_POSITION_TOLERANCE);
  }
}
