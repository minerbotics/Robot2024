package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class AutoDriveForward extends SequentialCommandGroup {
  private Swerve m_drivetrain;
    
    public AutoDriveForward(Swerve drivetrain) {
        m_drivetrain = drivetrain;
        addCommands(
            new AutoDrive(m_drivetrain, new ChassisSpeeds(0.7, 0, 0)).withTimeout(2),
            new AutoDrive(m_drivetrain, new ChassisSpeeds(0, 0, 0))
        );

        addRequirements(m_drivetrain);
    }
}
