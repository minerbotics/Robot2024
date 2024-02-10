// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.GoalTypeConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DoShoot extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake m_intake;
  private final Shooter m_shooter;
  private final int m_goalType; 

  /**
   * Creates a new DoShoot.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DoShoot(Intake intake, Shooter shooter, int goalType) {
    m_intake = intake;
    m_shooter = shooter;
    m_goalType = goalType;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

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
