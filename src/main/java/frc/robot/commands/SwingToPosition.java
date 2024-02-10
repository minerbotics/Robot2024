// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.GoalTypeConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Swinger;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SwingToPosition extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Swinger m_Swinger;
  private final int m_goalType;

  /**
   * Creates a new SwingToPosition.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SwingToPosition(Swinger swinger, int goalType) {
    m_Swinger = swinger;
    m_goalType = goalType;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swinger);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
