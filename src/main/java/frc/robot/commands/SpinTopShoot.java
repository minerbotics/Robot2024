package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

public class SpinTopShoot extends Command {
  private final Shooter m_shooter;

  public SpinTopShoot(Shooter shooter) {
    m_shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.spinTop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
  }
}
