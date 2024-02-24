package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Swinger;

public class ManualSwing extends Command {
    private final Swinger m_Swinger;

    private CommandXboxController m_Controller;

    public ManualSwing(Swinger swinger, CommandXboxController controller) {
        m_Swinger = swinger;
        m_Controller = controller;
        addRequirements(swinger);
    }

    @Override
    public void execute() {
        m_Swinger.move(m_Controller.getLeftY() * 0.75);
    }

    @Override
    public void end(boolean interrupted) {
        m_Swinger.stop();
    }
}
