// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private final CANSparkMax m_IntakeMotor;

  /** Creates a new ExampleSubsystem. */
  public Intake() {
    m_IntakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void in() {
    m_IntakeMotor.set(0.5);
  }

  public void stop() {
    m_IntakeMotor.set(0);
  }

  public void out() {
    m_IntakeMotor.set(-0.5);
  }
}
