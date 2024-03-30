package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private final CANSparkMax m_MidShooterMotor, m_BottomShooterMotor, m_TopRightShooterMotor, m_TopLeftShooterMotor;

  public Shooter() {
    m_MidShooterMotor = new CANSparkMax(ShooterConstants.MID_SHOOTER_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    m_BottomShooterMotor = new CANSparkMax(ShooterConstants.BOTTOM_SHOOTER_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    m_TopRightShooterMotor = new CANSparkMax(ShooterConstants.TOP_RIGHT_SHOOTER_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    m_TopLeftShooterMotor = new CANSparkMax(ShooterConstants.TOP_LEFT_SHOOTER_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    m_TopRightShooterMotor.follow(m_TopLeftShooterMotor, true);
  }

  public void ampOut() {
    m_MidShooterMotor.set(0.25);
    m_BottomShooterMotor.set(0.25);
  }

  public void speakerOut() {
    m_TopLeftShooterMotor.set(-1);
    m_MidShooterMotor.set(-0.5);
    m_BottomShooterMotor.set(-0.5);
  }

  public void intakeIn() {
    m_BottomShooterMotor.set(-1);
    m_MidShooterMotor.set(-0.15);
    m_TopLeftShooterMotor.set(0);
  }

  public void spinTop() {
    m_TopLeftShooterMotor.set(-1);
  }

  public void stop() {
    m_TopRightShooterMotor.set(0);
    m_BottomShooterMotor.set(0);
    m_MidShooterMotor.set(0);
    m_TopLeftShooterMotor.set(0);
  }
}
