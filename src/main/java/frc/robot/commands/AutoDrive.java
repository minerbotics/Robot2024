package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class AutoDrive extends Command {
  private Swerve m_Swerve;
  private ChassisSpeeds m_ChassisSpeeds;

  public AutoDrive(Swerve swerve, ChassisSpeeds chassisSpeeds) {
    m_Swerve = swerve;
    m_ChassisSpeeds = chassisSpeeds;
    addRequirements(m_Swerve);
  }

  @Override
  public void execute() {
    m_Swerve.drive(m_ChassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    m_Swerve.drive(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
