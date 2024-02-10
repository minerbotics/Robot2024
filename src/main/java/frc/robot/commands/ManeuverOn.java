// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.GoalTypeConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ManeuverOn extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Swerve m_Swerve;
  private final Limelight m_Limelight;
  private final int m_goalType; 

  public ManeuverOn(Swerve swerve, Limelight limelight, int goalType) {
    m_Swerve = swerve;
    m_Limelight = limelight;
    m_goalType = goalType; 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve, limelight);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(m_goalType){
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
