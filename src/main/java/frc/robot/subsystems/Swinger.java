package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwingConstants;

public class Swinger extends SubsystemBase {

  private CANSparkMax m_LeftSwingMotor, m_RightSwingMotor;
  private SparkPIDController m_PidController;
  private RelativeEncoder m_Encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;


  public Swinger() {
    m_LeftSwingMotor = new CANSparkMax(SwingConstants.LEFT_SWING_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    m_RightSwingMotor = new CANSparkMax(SwingConstants.RIGHT_SWING_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    m_RightSwingMotor.follow(m_LeftSwingMotor, true);

    m_PidController = m_LeftSwingMotor.getPIDController();
    m_Encoder = m_LeftSwingMotor.getEncoder();

    m_LeftSwingMotor.restoreFactoryDefaults();
    m_RightSwingMotor.restoreFactoryDefaults();

    // PID coefficients
    kP = 1.5;
    kI = 1e-4;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 0.5;
    kMinOutput = -0.5;

    // set PID coefficients
    m_PidController.setP(kP);
    m_PidController.setI(kI);
    m_PidController.setD(kD);
    m_PidController.setIZone(kIz);
    m_PidController.setFF(kFF);
    m_PidController.setOutputRange(kMinOutput, kMaxOutput);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Swinger Position", m_Encoder.getPosition());
  }

  public void swingToPosition(double position) {
    double rotations = degreesToRotation(position);
    m_PidController.setReference(rotations, ControlType.kPosition);
  }

  public double getPosition() {
    return m_Encoder.getPosition();
  }

  public void move(double speed) {
    m_LeftSwingMotor.set(speed*0.5);
  }

  public void stop() {
    m_LeftSwingMotor.set(0);
  }

  private double degreesToRotation(double degrees) {
    return (degrees / 360);
  }
}
