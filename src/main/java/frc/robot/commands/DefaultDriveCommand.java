package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class DefaultDriveCommand extends Command {
  private final Swerve m_swerve;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final boolean m_isRobotCentric;

    public DefaultDriveCommand(Swerve swerve,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.m_swerve = swerve;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.m_isRobotCentric = false;

        addRequirements(swerve);
    }

    public DefaultDriveCommand(Swerve swerve,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier,
                               boolean isRobotCentric) {
        this.m_isRobotCentric = isRobotCentric;
        this.m_swerve = swerve;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        ChassisSpeeds chassisSpeeds;
        if (!m_isRobotCentric) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        -m_translationXSupplier.getAsDouble(),
                        -m_translationYSupplier.getAsDouble(),
                        -m_rotationSupplier.getAsDouble(),
                        m_swerve.getGyroscopeRotation());
        } else {
            chassisSpeeds = new ChassisSpeeds(
                -m_translationXSupplier.getAsDouble(), 
                -m_translationYSupplier.getAsDouble(),
                -m_rotationSupplier.getAsDouble());
        }
        m_swerve.drive(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
