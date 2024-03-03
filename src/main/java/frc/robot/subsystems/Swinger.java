package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwingConstants;

public class Swinger extends SubsystemBase {

  private final int kCPR = 8192;

  private CANSparkMax m_LeftSwingMotor, m_RightSwingMotor;
  private SparkPIDController m_PidController;
  private RelativeEncoder m_Encoder;
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
    m_Encoder = m_RightSwingMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, kCPR);
    m_Encoder.setInverted(true);
    m_PidController.setFeedbackDevice(m_Encoder);

    // PID coefficients
    kP = 1;
    kI = 0;
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
    
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Alt Encoder Position", this.getPosition());
    SmartDashboard.putBoolean("Encoder Position Conversion", m_Encoder.getInverted());
    SmartDashboard.putNumber("Alt Encoder Velocity", m_Encoder.getVelocity());
    SmartDashboard.putNumber("Applied Output", m_RightSwingMotor.getAppliedOutput());

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_PidController.setP(p); kP = p; }
    if((i != kI)) { m_PidController.setI(i); kI = i; }
    if((d != kD)) { m_PidController.setD(d); kD = d; }
    if((iz != kIz)) { m_PidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_PidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_PidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
  }

  public void swingToPosition(double position) {
    double rotations = degreesToRotation(position);
    SmartDashboard.putNumber("Set Point", rotations);
    SmartDashboard.putNumber("Set Point Degrees", position);
    m_PidController.setReference(rotations, ControlType.kPosition);
  }

  public double getPosition() {
    return (m_Encoder.getPosition() * 360);
  }

  public void move(double speed) {
    m_RightSwingMotor.set(speed*0.35);
  }

  public void stop() {
    m_RightSwingMotor.set(0);
  }

  private double degreesToRotation(double degrees) {
    return (degrees / 360);
  }
}
