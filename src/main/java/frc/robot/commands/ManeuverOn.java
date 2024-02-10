package frc.robot.commands;

import frc.robot.Constants.GoalTypeConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class ManeuverOn extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Swerve m_Swerve;
  private final Limelight m_Limelight;
  private final int m_goalType;

  private double txMin, txMax, taMin, taMax;
  private boolean m_isInPosition;

  public ManeuverOn(Swerve swerve, Limelight limelight, int goalType) {
    m_Swerve = swerve;
    m_Limelight = limelight;
    m_goalType = goalType; 
    m_isInPosition = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve, limelight);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(m_goalType){
      case GoalTypeConstants.AMP:
        txMin = -4;
        txMax = 4;
        taMin = 2;
        taMax = 3;
        break;
      case GoalTypeConstants.SPEAKER:
        txMin = -4;
        txMax = 4;
        taMin = 2;
        taMax = 3;
        break;
      case GoalTypeConstants.SOURCE_1:
        txMin = -4;
        txMax = 4;
        taMin = 2;
        taMax = 3;
        break;
      case GoalTypeConstants.SOURCE_2:
        txMin = -4;
        txMax = 4;
        taMin = 2;
        taMax = 3;
        break;
      case GoalTypeConstants.SOURCE_3:
        txMin = -4;
        txMax = 4;
        taMin = 2;
        taMax = 3;
        break;
    }
    moveWithCamera(txMin, txMax, taMin, taMax);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isInPosition;
  }

  private void moveWithCamera(double txMin, double txMax, double taMin, double taMax) {
    double tx = m_Limelight.getTX();
    double ta = m_Limelight.getTA();
    double tv = m_Limelight.getTV();

    double vx = 0.0;
    double vy = 0.0;
    double omega = 0.0;

    boolean inRange = false;
    boolean linedUp = false;

    if (tv < 1.0) {
      m_Swerve.drive(new ChassisSpeeds(vx, vy, omega));
      return;
    }

    if (tx > txMax || tx < txMin) {
      vy = tx * 0.05;
    } else {
      vy = 0;
      linedUp = true;
    }

    if (ta > taMax || ta < taMin) {
      vx = (ta - ((taMax + taMin)/2)) * 0.5;
    } else {
      vx = 0;
      inRange = true;
    }

    if(linedUp && inRange) {
      m_isInPosition = true;
    }

    m_Swerve.drive(new ChassisSpeeds(vx, vy, omega));
  }
}
