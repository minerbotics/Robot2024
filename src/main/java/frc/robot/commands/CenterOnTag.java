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
        //System.out.println(tx);
        if (tx > 4) {
            if (ta > 3) {
                m_Swerve.drive(new ChassisSpeeds(0.5, 0.5, 0));
            } else if (ta < 2) {
               m_Swerve.drive(new ChassisSpeeds(-0.5, 0.5, 0));
               System.out.println("test12");
            } else {
                m_Swerve.drive(new ChassisSpeeds(0, 0.5, 0));
            }
        } else if (tx < -4) {
            if (ta > 3) {
                m_Swerve.drive(new ChassisSpeeds(0.5, -0.5, 0));
            } else if (ta < 2) {
                m_Swerve.drive(new ChassisSpeeds(-0.5, -0.5, 0));
                System.out.println("test11");
            } else {
                m_Swerve.drive(new ChassisSpeeds(0, -0.5, 0));
            }
        } else if (ta > 3) {
            m_Swerve.drive(new ChassisSpeeds(0.5, 0, 0));
        } else if (ta < 2) {
               m_Swerve.drive(new ChassisSpeeds(-0.5, 0, 0));
        }
        
        else {
            m_Swerve.drive(new ChassisSpeeds(0, 0, 0));
        }

    }
}
