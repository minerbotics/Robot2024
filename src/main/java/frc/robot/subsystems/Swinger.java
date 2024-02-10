// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwingConstants;

public class Swinger extends SubsystemBase {

  private final CANSparkMax m_LeftSwingMotor, m_RightSwingMotor;

  /** Creates a new ExampleSubsystem. */
  public Swinger() {
    m_LeftSwingMotor = new CANSparkMax(SwingConstants.LEFT_SWING_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    m_RightSwingMotor = new CANSparkMax(SwingConstants.RIGHT_SWING_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    m_RightSwingMotor.follow(m_LeftSwingMotor, true);
  }


}
