package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.GoalTypeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swinger;

public class AutoAmpScore extends SequentialCommandGroup {
    private Swerve m_Swerve;
    private Swinger m_Swinger;
    private IntakeSubsystem m_Intake;
    private Shooter m_Shooter;
    
    public AutoAmpScore(Swerve swerve, Swinger swinger, IntakeSubsystem intake, Shooter shooter){
        m_Swerve = swerve;
        m_Swinger = swinger;
        m_Intake = intake;
        m_Shooter = shooter;
        addCommands(
            new ManeuverOn(m_Swerve, GoalTypeConstants.AMP).withTimeout(3),
            new SwingToPosition(m_Swinger, GoalTypeConstants.AMP),
            new DoShoot(m_Intake, m_Shooter, GoalTypeConstants.AMP).withTimeout(1),
            new AutoDrive(m_Swerve, new ChassisSpeeds(0, 0, 0.5)).withTimeout(1),
            new AutoDrive(m_Swerve, new ChassisSpeeds(-0.5, 0, 0)).withTimeout(3)
        );
    }
}
