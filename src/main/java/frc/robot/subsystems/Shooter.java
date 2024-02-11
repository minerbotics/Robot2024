package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private final CANSparkMax m_TopShooterMotor, m_MidShooterMotor, m_BottomShooterMotor;

  public Shooter() {
    m_TopShooterMotor = new CANSparkMax(ShooterConstants.TOP_SHOOTER_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    m_MidShooterMotor = new CANSparkMax(ShooterConstants.MID_SHOOTER_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    m_BottomShooterMotor = new CANSparkMax(ShooterConstants.BOTTOM_SHOOTER_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
  }

  public void ampOut() {
    m_BottomShooterMotor.set(0.5);
    m_MidShooterMotor.set(0.5);
  }

  public void speakerOut() {
    m_TopShooterMotor.set(-1);
    m_MidShooterMotor.set(-1);
  }

  public void intakeIn() {
    m_BottomShooterMotor.set(-0.5);
    m_MidShooterMotor.set(-0.5);
    m_TopShooterMotor.set(0);
  }

  public void stop() {
    m_BottomShooterMotor.set(0);
    m_MidShooterMotor.set(0);
    m_TopShooterMotor.set(0);
  }
}
