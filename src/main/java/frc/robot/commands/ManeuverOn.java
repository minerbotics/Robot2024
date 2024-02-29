package frc.robot.commands;

import frc.robot.Constants.GoalTypeConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

import java.util.Optional;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class ManeuverOn extends Command {
  private final Swerve m_Swerve;
  private final Limelight m_Limelight;
  private final int m_goalType;

  private double txMin, txMax, taMin, taMax;
  private boolean m_isInPosition, m_checkTarget;

  public ManeuverOn(Swerve swerve, int goalType) {
    m_Swerve = swerve;
    m_Limelight = new Limelight();
    m_goalType = goalType; 
    m_isInPosition = false;
    m_checkTarget = true;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  public ManeuverOn(Swerve swerve, int goalType, boolean checkTarget) {
    this(swerve, goalType);
    m_checkTarget = checkTarget;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(m_goalType){
      case GoalTypeConstants.AMP:
        txMin = -2;
        txMax = 2;
        taMin = 5;
        taMax = 6;
        break;
      case GoalTypeConstants.SPEAKER:
        txMin = -2;
        txMax = 2;
        taMin = .75;
        taMax = .9;
        break;
      case GoalTypeConstants.SOURCE_1:
        txMin = -2;
        txMax = 2;
        taMin = 5;
        taMax = 6;
        break;
      case GoalTypeConstants.SOURCE_2:
        txMin = -2;
        txMax = 2;
        taMin = 5;
        taMax = 6;
        break;
      case GoalTypeConstants.SOURCE_3:
        txMin = -2;
        txMax = 2;
        taMin = 5;
        taMax = 6;
        break;
    }
    double targetId = whichAprilTag(m_goalType);
    moveWithCamera(txMin, txMax, taMin, taMax, targetId);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isInPosition;
  }

  @Override
  public void end(boolean interrupted) {
    m_Swerve.rawDrive(
      new ChassisSpeeds(
        0,
        0, 
        0));
  }

  // Calls the drive command with ChassisSpeeds based on how close
  // the target is and how far to the left or right the target is
  // and whether the specified target is in view.
  private void moveWithCamera(double txMin, double txMax, double taMin, double taMax, double targetId) {
    double tx = m_Limelight.getTX();
    double ta = m_Limelight.getTA();
    double tv = m_Limelight.getTV();

    double vx = 0.0;
    double vy = 0.0;
    double omega = 0.0;

    boolean inRange = false;
    boolean linedUp = false;

    if (tv < 1.0 || (m_checkTarget && m_Limelight.getTargetId() != targetId)) {
      m_Swerve.rawDrive(new ChassisSpeeds(vx, vy, omega));
      return;
    }

    if (tx > txMax || tx < txMin) {
      vy = tx * 0.035;
      linedUp = false;
    } else {
      vy = 0;
      linedUp = true;
    }

    if (ta > taMax) {
      // Forward
      vx = 0.25;
      linedUp = false;
    } else if (ta < taMin) {
      // Backward
      vx = -0.25;
      linedUp = false;
    } else {
      vx = 0;
      inRange = true;
    }

    if(linedUp && inRange) {
      m_isInPosition = true;
    } else {
      m_isInPosition = false;
    }

    m_Swerve.rawDrive(new ChassisSpeeds(vx, vy, omega));
  }

  // Returns which april tag we want to target depending on which 
  // alliance we are on and which goal we are trying to maneuver on.
  private double whichAprilTag(int goalType) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    double tagId = 0;
    if(alliance.get() == DriverStation.Alliance.Red) {
      if (goalType == GoalTypeConstants.AMP) {
        tagId = 5;
      } else if(goalType == GoalTypeConstants.SPEAKER) {
        tagId = 4;
      } else if(goalType == GoalTypeConstants.SOURCE_1) {
        tagId = 9;
      } else if(goalType == GoalTypeConstants.SOURCE_3) {
        tagId = 10;
      } 
    } else {
      if (goalType == GoalTypeConstants.AMP) {
        tagId = 6;
      } else if(goalType == GoalTypeConstants.SPEAKER) {
        tagId = 7;
      } else if(goalType == GoalTypeConstants.SOURCE_1) {
        tagId = 1;
      } else if(goalType == GoalTypeConstants.SOURCE_3) {
        tagId = 2;
      } 
    }
    return tagId;
  }
}
