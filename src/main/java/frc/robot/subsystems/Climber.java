package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private final DoubleSolenoid m_LeftClimber, m_RightClimber;

  public Climber() {
    m_RightClimber = new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM, 
      ClimberConstants.RIGHT_CLIMB_DOWN_CHANNEL, 
      ClimberConstants.RIGHT_CLIMB_UP_CHANNEL);

    m_LeftClimber = new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM, 
      ClimberConstants.LEFT_CLIMB_DOWN_CHANNEL, 
      ClimberConstants.LEFT_CLIMB_UP_CHANNEL);
  }

  public void up() {
    m_RightClimber.set(Value.kReverse);
    m_LeftClimber.set(Value.kReverse);
  }

  public void down() {
    m_RightClimber.set(Value.kForward);        
    m_LeftClimber.set(Value.kForward);
  }

  public boolean isUp() {
    return m_LeftClimber.get() == Value.kReverse;
  }
}
