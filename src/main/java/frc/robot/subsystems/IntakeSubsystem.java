package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkMax m_IntakeMotor;

  public IntakeSubsystem() {
    m_IntakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void in() {
    m_IntakeMotor.set(-0.5);
  }

  public void stop() {
    m_IntakeMotor.set(0);
  }

  public void out() {
    m_IntakeMotor.set(0.5);
  }
}
