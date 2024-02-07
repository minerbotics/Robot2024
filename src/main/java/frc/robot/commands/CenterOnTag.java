package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class CenterOnTag extends Command {
    private Limelight m_Limelight;
    private Swerve m_Swerve;

    public CenterOnTag(Limelight limelight, Swerve swerve) {
        m_Limelight = limelight;
        m_Swerve = swerve;
        addRequirements(limelight, swerve);
    }

    @Override
    public void execute() {
        double tx = m_Limelight.getTX();
        double ta = m_Limelight.getTA();
        
        double vx = 0.0;
        double vy = 0.0;
        double omega = 0.0;

        if (tx > 4) {
            vy = 0.5;
        } else if (tx < -4) {
            vy = -0.5;
        }

        if (ta > 3) {
            vx = 0.5;
        } else if (ta < 2) {
            vx = -0.5;
        }

        m_Swerve.drive(new ChassisSpeeds(vx, vy, omega));
    }
}
