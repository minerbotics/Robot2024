package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberUp extends Command {
  private final Climber m_Climber;
  
  public ClimberUp(Climber climber) {
    m_Climber = climber;

    addRequirements(climber);
  }

  @Override
  public void execute() {
    m_Climber.up();
  }
}
