// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private final CANSparkMax m_TopShooterMotor, m_MidShooterMotor, m_BottomShooterMotor;

  /** Creates a new ExampleSubsystem. */
  public Shooter() {
    m_TopShooterMotor = new CANSparkMax(ShooterConstants.TOP_SHOOTER_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    m_MidShooterMotor = new CANSparkMax(ShooterConstants.MID_SHOOTER_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    m_BottomShooterMotor = new CANSparkMax(ShooterConstants.BOTTOM_SHOOTER_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
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
