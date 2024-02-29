package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private final DoubleSolenoid m_Climber;

  public Climber() {

    m_Climber = new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM, 
      ClimberConstants.CLIMBER_DOWN_CHANNEL, 
      ClimberConstants.CLIMBER_UP_CHANNEL);
  }

  public void up() {
    m_Climber.set(Value.kReverse);
  }

  public void down() {
    m_Climber.set(Value.kForward);
  }

  public boolean isUp() {
    return m_Climber.get() == Value.kReverse;
  }
}
