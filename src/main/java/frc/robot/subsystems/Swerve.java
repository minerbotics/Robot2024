package frc.robot.subsystems;

import com.swervedrivespecialties.swervelib.MkModuleConfiguration;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.*;

import java.util.Optional;

public class Swerve extends SubsystemBase {
  ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
  /** 
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;

  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 5880.0 / 60.0 *
    SdsModuleConfigurations.MK4_L2.getDriveReduction() *
    SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  public MkModuleConfiguration swerveConfig;

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  //private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP
  private final Pigeon2 pigeon2;

  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private final int steerLimit = 20;
  private final int driveLimit = 30;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  private final SlewRateLimiter m_slewX;
  private final SlewRateLimiter m_slewY;
  private final SlewRateLimiter m_slewRot;

  private final SwerveDrivePoseEstimator m_PoseEstimator;
  private final Pose2d m_StartingPose;

  public Swerve() {
    pigeon2 = new Pigeon2(PIGEON_ID);

    swerveConfig = MkModuleConfiguration.getDefaultSteerNEO();
    swerveConfig.setDriveCurrentLimit(driveLimit);
    swerveConfig.setSteerCurrentLimit(steerLimit);

    m_frontLeftModule = new MkSwerveModuleBuilder(swerveConfig)
      .withLayout(tab.getLayout("Front left module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0))
      .withGearRatio(SdsModuleConfigurations.MK4_L2)
      .withDriveMotor(MotorType.NEO, FRONT_LEFT_MODULE_DRIVE_MOTOR)
      .withSteerMotor(MotorType.NEO, FRONT_LEFT_MODULE_STEER_MOTOR)
      .withSteerEncoderPort(FRONT_LEFT_MODULE_STEER_ENCODER)
      .withSteerOffset(FRONT_LEFT_MODULE_STEER_OFFSET)
      .build();

    m_frontRightModule = new MkSwerveModuleBuilder(swerveConfig)
      .withLayout(tab.getLayout("Front right module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0))
      .withGearRatio(SdsModuleConfigurations.MK4_L2)
      .withDriveMotor(MotorType.NEO, FRONT_RIGHT_MODULE_DRIVE_MOTOR)
      .withSteerMotor(MotorType.NEO, FRONT_RIGHT_MODULE_STEER_MOTOR)
      .withSteerEncoderPort(FRONT_RIGHT_MODULE_STEER_ENCODER)
      .withSteerOffset(FRONT_RIGHT_MODULE_STEER_OFFSET)
      .build();

    m_backLeftModule = new MkSwerveModuleBuilder(swerveConfig)
      .withLayout(tab.getLayout("Back left module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0))
      .withGearRatio(SdsModuleConfigurations.MK4_L2)
      .withDriveMotor(MotorType.NEO, BACK_LEFT_MODULE_DRIVE_MOTOR)
      .withSteerMotor(MotorType.NEO, BACK_LEFT_MODULE_STEER_MOTOR)
      .withSteerEncoderPort(BACK_LEFT_MODULE_STEER_ENCODER)
      .withSteerOffset(BACK_LEFT_MODULE_STEER_OFFSET)
      .build();

    m_backRightModule = new MkSwerveModuleBuilder(swerveConfig)
      .withLayout(tab.getLayout("Back right module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0))
      .withGearRatio(SdsModuleConfigurations.MK4_L2)
      .withDriveMotor(MotorType.NEO, BACK_RIGHT_MODULE_DRIVE_MOTOR)
      .withSteerMotor(MotorType.NEO, BACK_RIGHT_MODULE_STEER_MOTOR)
      .withSteerEncoderPort(BACK_RIGHT_MODULE_STEER_ENCODER)
      .withSteerOffset(BACK_RIGHT_MODULE_STEER_OFFSET)
      .build();

    m_slewX = new SlewRateLimiter(Constants.TRANSLATION_SLEW);
    m_slewY = new SlewRateLimiter(Constants.TRANSLATION_SLEW);
    m_slewRot = new SlewRateLimiter(Constants.ROTATION_SLEW);

    m_StartingPose = new Pose2d(0, 0, new Rotation2d(0));
    m_PoseEstimator = new SwerveDrivePoseEstimator(m_kinematics, getGyroscopeRotation(), getModulePositions(), m_StartingPose);

    AutoBuilder.configureHolonomic(
                m_PoseEstimator::getEstimatedPosition, // Robot pose supplier
                this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::rawDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        AutoConstants.TRANSLATION_AUTO_PID, // Translation PID constants
                        AutoConstants.ROTATION_AUTO_PID, // Rotation PID constants
                        MAX_VELOCITY_METERS_PER_SECOND , // Max module speed, in m/s
                        DRIVETRAIN_WHEELBASE_RADIUS, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig(true, false) // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    Optional<Alliance> alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    pigeon2.reset();
  }

  public double getPitchAngleDegrees() {
        return pigeon2.getPitch().getValueAsDouble();
  }

  public double getRollAngleDegrees() {
        return pigeon2.getRoll().getValueAsDouble();
  }

  public Rotation2d getGyroscopeRotation() {
    return pigeon2.getRotation2d();
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = new ChassisSpeeds(
      m_slewX.calculate(chassisSpeeds.vxMetersPerSecond),
      m_slewY.calculate(chassisSpeeds.vyMetersPerSecond),
      m_slewRot.calculate(chassisSpeeds.omegaRadiansPerSecond)
    );
  }

  public void rawDrive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  @Override
  public void periodic() {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    
    this.m_PoseEstimator.update(getGyroscopeRotation(), getModulePositions());
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = {
      m_frontLeftModule.getPosition(),
      m_frontRightModule.getPosition(),
      m_backLeftModule.getPosition(),
      m_backRightModule.getPosition()
    };
    return positions;
  }

  public void setPose(Pose2d pose) {
    m_PoseEstimator.resetPosition(getGyroscopeRotation(), getModulePositions(), pose);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getModuleStates());
  }

  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] swerveModuleStates = {
      m_frontLeftModule.getState(),
      m_frontRightModule.getState(),
      m_backLeftModule.getState(),
      m_backRightModule.getState()
    };
    return swerveModuleStates;
  }

}
