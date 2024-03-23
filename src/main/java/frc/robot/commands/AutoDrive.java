package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class AutoDrive extends Command {
  private Swerve m_Swerve;
  private ChassisSpeeds m_ChassisSpeeds;
  private boolean m_ShouldSetColor;

  public AutoDrive(Swerve swerve, ChassisSpeeds chassisSpeeds) {
    m_Swerve = swerve;
    m_ChassisSpeeds = chassisSpeeds;
    m_ShouldSetColor = false;
    addRequirements(m_Swerve);
  }

  public AutoDrive(Swerve swerve, ChassisSpeeds chassisSpeeds, boolean shouldSetColor) {
    m_Swerve = swerve;
    m_ChassisSpeeds = chassisSpeeds;
    m_ShouldSetColor = shouldSetColor;
    addRequirements(m_Swerve);
  }

  @Override
  public void execute() {
    
    Optional<Alliance> alliance = DriverStation.getAlliance();
    int scalar = 1;
    if (m_ShouldSetColor && alliance.get() == DriverStation.Alliance.Red) {
      scalar = -1;
    } 
     
    m_Swerve.drive(m_ChassisSpeeds.times(scalar));
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
