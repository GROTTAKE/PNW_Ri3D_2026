// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import com.studica.frc.AHRS;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private static final boolean DEMO_PUBLISH_FAKE_POSE = false;
  // Drivetrain & odometry configuration (update to match your robot)
  private static final int LEFT_MOTOR_CAN_ID = 2;  // per your setup
  private static final int RIGHT_MOTOR_CAN_ID = 1; // per your setup
  private double m_wheelDiameterMeters = 0.1524; // 6 in wheel
  // Motor revs per wheel rev; set from your gearbox ratio
  private double m_driveGearRatio = 10.71;
  // Conversion factors (computed from the two values above)
  private double m_metersPerMotorRev = (Math.PI * m_wheelDiameterMeters) / m_driveGearRatio;
  private double m_metersPerSecPerRPM = m_metersPerMotorRev / 60.0;

  private final DifferentialDrive m_robotDrive;
  private final XboxController m_controller;

  private final SparkMax m_leftMotor =
      new SparkMax(LEFT_MOTOR_CAN_ID, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax m_rightMotor =
      new SparkMax(RIGHT_MOTOR_CAN_ID, SparkLowLevel.MotorType.kBrushless);
  private final Field2d m_field = new Field2d();
  private final Timer m_poseDemoTimer = new Timer();
  //private final AHRS m_navx = new AHRS(SPI.Port.kMXP);
  private final AHRS m_navx = new AHRS(AHRS.NavXComType.kMXP_SPI);
  private final StructPublisher<Pose2d> m_pose2dPublisher =
      NetworkTableInstance.getDefault()
          .getStructTopic("/Visualization/RobotPose2d", Pose2d.struct)
          .publish();
  private final RelativeEncoder m_leftEncoder = m_leftMotor.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightMotor.getEncoder();
  private final DifferentialDriveOdometry m_odometry =
      new DifferentialDriveOdometry(new Rotation2d(), 0.0, 0.0);

  /** Called once at the beginning of the robot program. */
  public Robot() {
    // Recommended current limits and idle mode can be set via configs in configureEncoderConversions().
    // m_leftMotor.setSmartCurrentLimit(50);
    // m_rightMotor.setSmartCurrentLimit(50);
    // m_leftMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    // m_rightMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

    m_robotDrive = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);
    m_controller = new XboxController(0);

    SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor);

    // Publish a Field2d to NetworkTables so dashboards (e.g., AdvantageScope) can visualize pose.
    SmartDashboard.putData("Field", m_field);
    // Configure REV encoders to report meters and m/s
    configureEncoderConversions();
    // Ensure forward on both sides increases position positively; flip sign by inverting motor if needed
    if (DEMO_PUBLISH_FAKE_POSE) {
      m_poseDemoTimer.restart();
    }
  }

  @Override
  public void robotInit() {
    // Zero navX yaw at startup; you can also call setInitialPoseMeters(...) to align to field heading.
    m_navx.zeroYaw();
    m_leftEncoder.setPosition(0.0);
    m_rightEncoder.setPosition(0.0);
    Rotation2d gyro = Rotation2d.fromDegrees(-m_navx.getYaw());
    m_odometry.resetPosition(gyro, m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), new Pose2d());
  }

  @Override
  public void robotPeriodic() {
    // If you don't have sensors yet, publish a harmless, animated "fake" pose so you can
    // confirm AdvantageScope is connected and rendering the field widget.
    if (DEMO_PUBLISH_FAKE_POSE) {
      double t = m_poseDemoTimer.get();
      double radiusMeters = 1.0;
      double xMeters = radiusMeters * Math.cos(t * 0.25);
      double yMeters = radiusMeters * Math.sin(t * 0.25);
      Rotation2d heading = Rotation2d.fromRadians(t * 0.25);
      Pose2d pose = new Pose2d(xMeters, yMeters, heading);
      m_field.setRobotPose(pose);
      m_pose2dPublisher.set(pose);
    } else {
      // navX yaw is positive clockwise; WPILib standard expects CCW-positive.
      Rotation2d gyro = Rotation2d.fromDegrees(-m_navx.getYaw());
      Pose2d pose = m_odometry.update(gyro, m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
      m_field.setRobotPose(pose);
      m_pose2dPublisher.set(pose);
    }
  }

  @Override
  public void teleopPeriodic() {
    // Tank drive using Xbox controller sticks
    m_robotDrive.tankDrive(-m_controller.getLeftY(), -m_controller.getRightY());
  }

  /** Set odometry to a specific field pose. Resets encoders and aligns to the desired pose. */
  public void setInitialPoseMeters(double xMeters, double yMeters, double headingDegrees) {
    m_leftEncoder.setPosition(0.0);
    m_rightEncoder.setPosition(0.0);
    Rotation2d gyro = Rotation2d.fromDegrees(-m_navx.getYaw());
    Pose2d desired = new Pose2d(xMeters, yMeters, Rotation2d.fromDegrees(headingDegrees));
    m_odometry.resetPosition(gyro, m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), desired);
  }

  // Recompute and apply encoder conversion factors from current wheel diameter and gear ratio.
  private void configureEncoderConversions() {
    m_metersPerMotorRev = (Math.PI * m_wheelDiameterMeters) / m_driveGearRatio;
    m_metersPerSecPerRPM = m_metersPerMotorRev / 60.0;
    SparkMaxConfig leftCfg = new SparkMaxConfig();
    leftCfg.inverted(false);
    leftCfg.encoder.positionConversionFactor(m_metersPerMotorRev);
    leftCfg.encoder.velocityConversionFactor(m_metersPerSecPerRPM);
    m_leftMotor.configure(leftCfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkMaxConfig rightCfg = new SparkMaxConfig();
    rightCfg.inverted(true); // invert right side so forward is positive on both
    rightCfg.encoder.positionConversionFactor(m_metersPerMotorRev);
    rightCfg.encoder.velocityConversionFactor(m_metersPerSecPerRPM);
    m_rightMotor.configure(rightCfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  // Call this to change the motor-to-wheel gear ratio at runtime (e.g., after measuring).
  public void setDriveGearRatio(double motorRevsPerWheelRev) {
    m_driveGearRatio = motorRevsPerWheelRev;
    configureEncoderConversions();
  }
}
