package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swinger;

public class ManualSwing extends Command {
    private final Swinger m_Swinger;

    private double m_Speed;

    public ManualSwing(Swinger swinger, double speed) {
        m_Swinger = swinger;
        m_Speed = speed;
        addRequirements(swinger);
    }

    @Override
    public void execute() {
        m_Swinger.move(m_Speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_Swinger.stop();
    }
}
