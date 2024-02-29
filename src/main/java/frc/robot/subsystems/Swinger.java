package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwingConstants;

public class Swinger extends SubsystemBase {

  private final int kCPR = 8192;

  private CANSparkMax m_LeftSwingMotor, m_RightSwingMotor;
  private SparkPIDController m_PidController;
  private DutyCycleEncoder m_Encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;


  public Swinger() {
    m_LeftSwingMotor = new CANSparkMax(SwingConstants.LEFT_SWING_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    m_RightSwingMotor = new CANSparkMax(SwingConstants.RIGHT_SWING_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    m_LeftSwingMotor.restoreFactoryDefaults();
    m_RightSwingMotor.restoreFactoryDefaults();
    m_RightSwingMotor.setInverted(true);
    m_LeftSwingMotor.follow(m_RightSwingMotor, true);
    m_RightSwingMotor.setIdleMode(IdleMode.kBrake);
    m_LeftSwingMotor.setIdleMode(IdleMode.kBrake);

    m_PidController = m_RightSwingMotor.getPIDController();
//    m_Encoder = m_RightSwingMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, kCPR);
      DigitalInput dio = new DigitalInput(SwingConstants.DIO_PORT);
      m_Encoder = new DutyCycleEncoder(dio);
//    m_PidController.setFeedbackDevice(m_Encoder);

    // PID coefficients
    kP = 1;
    kI = 1e-4;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

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
//    SmartDashboard.putNumber("Alt Encoder Position", m_Encoder.getPosition());
//    SmartDashboard.putBoolean("Encoder Position Conversion", m_Encoder.getInverted());
//    SmartDashboard.putNumber("Alt Encoder Velocity", m_Encoder.getVelocity());
//    SmartDashboard.putNumber("Applied Output", m_RightSwingMotor.getAppliedOutput());
      SmartDashboard.putNumber("Absolute Encoder Position", m_Encoder.getAbsolutePosition());
  }

  public void swingToPosition(double position) {
    double rotations = degreesToRotation(position);
//    m_PidController.setReference(rotations, ControlType.kPosition);
  }

  public double getPosition() {
    return m_Encoder.getAbsolutePosition();
  }

  public void move(double speed) {
    m_RightSwingMotor.set(speed*0.5);
  }

  public void stop() {
    m_RightSwingMotor.set(0);
  }

  private double degreesToRotation(double degrees) {
    return (degrees / 360);
  }
}
