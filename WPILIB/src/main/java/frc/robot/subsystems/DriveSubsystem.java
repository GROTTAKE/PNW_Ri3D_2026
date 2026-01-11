package frc.robot.subsystems;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.studica.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Limelight;
import frc.robot.utils.LimelightHelpers;

public class DriveSubsystem extends SubsystemBase {
    
    private final AHRS NavX;

    private final SparkMax LeftLeadMotor;
    private final SparkMax LeftFollowMotor;
    private final SparkMax RightLeadMotor;
    private final SparkMax RightFollowMotor;

    private final SparkClosedLoopController LeftController;
    private final SparkClosedLoopController RightController;

    private final DifferentialDriveKinematics Kinematics;
    private final DifferentialDrivePoseEstimator PoseEstimator;

    private final StructPublisher<Pose2d> PosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("/Pose2D", Pose2d.struct)
            .publish();

    public DriveSubsystem() {
        this.NavX = new AHRS(AHRS.NavXComType.kMXP_SPI);

        // Main Left Motor

        this.LeftLeadMotor = new SparkMax(Constants.Drive.PORT_A_CAN_ID, MotorType.kBrushless);

        {
            var config = new SparkMaxConfig().idleMode(SparkBaseConfig.IdleMode.kBrake);

            config.encoder.positionConversionFactor(Constants.Drive.DISTANCE_PER_MOTOR_REV.in(Meters));
            config.encoder.velocityConversionFactor(Constants.Drive.DISTANCE_PER_MOTOR_REV.in(Meters) / 60);

            config.closedLoop.maxMotion.maxAcceleration(Constants.Drive.MAX_TRANSLATION_ACCEL_RPM.in(RPM));

            this.LeftLeadMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }

        // Main Right Motor

        this.RightLeadMotor = new SparkMax(Constants.Drive.STARBOARD_A_CAN_ID, SparkLowLevel.MotorType.kBrushless);

        {
            var config = new SparkMaxConfig().idleMode(SparkBaseConfig.IdleMode.kBrake);

            config.encoder.positionConversionFactor(Constants.Drive.DISTANCE_PER_MOTOR_REV.in(Meters));
            config.encoder.velocityConversionFactor(Constants.Drive.DISTANCE_PER_MOTOR_REV.in(Meters) / 60);

            config.closedLoop.maxMotion.maxAcceleration(Constants.Drive.MAX_TRANSLATION_ACCEL_RPM.in(RPM));

            this.RightLeadMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }

        // Follower Left Motor

        this.LeftFollowMotor = new SparkMax(Constants.Drive.PORT_B_CAN_ID, SparkLowLevel.MotorType.kBrushless);

        this.LeftFollowMotor.configure(
                new SparkMaxConfig().follow(LeftLeadMotor).idleMode(SparkBaseConfig.IdleMode.kBrake),
                ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // Follower Right Motor

        this.RightFollowMotor = new SparkMax(Constants.Drive.STARBOARD_B_CAN_ID, SparkLowLevel.MotorType.kBrushless);

        this.RightFollowMotor.configure(
                new SparkMaxConfig().follow(RightLeadMotor).idleMode(SparkBaseConfig.IdleMode.kBrake),
                ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // Closed Loop Controllers

        this.LeftController = this.LeftLeadMotor.getClosedLoopController();
        this.RightController = this.RightLeadMotor.getClosedLoopController();

        // Kinematics & Odometry

        this.Kinematics = new DifferentialDriveKinematics(Constants.Drive.TRACK_WIDTH);
        this.PoseEstimator = new DifferentialDrivePoseEstimator(
                this.Kinematics,
                new Rotation2d(),
                0,
                0,
                new Pose2d());

        try {

            AutoBuilder.configure(
                    this::getEstimatedPose,
                    (rPose) -> {
                    },
                    this::getMeasuredRobotRelativeChassisSpeeds,
                    (speeds, feed) -> this.driveRobotRelative(speeds),
                    new PPLTVController(Robot.kDefaultPeriod, Constants.Drive.MAX_TRANSLATION_AUTO_VELOCITY.in(MetersPerSecond)),
                    RobotConfig.fromGUISettings(),
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red
                        // alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this // Reference to this subsystem to set requirements
            );

        } catch (Exception e) {

            System.err.println("Failed to configure AutoBuilder!");
            e.printStackTrace();

        }
    }

    public Pose2d getEstimatedPose() {
        return this.PoseEstimator.getEstimatedPosition();
    }

    public Rotation2d getGyroRotation() {
        return this.NavX.getRotation2d();
    }

    public DifferentialDriveWheelSpeeds getMeasuredWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(this.LeftLeadMotor.getEncoder().getVelocity(),
                this.RightLeadMotor.getEncoder().getVelocity());
    }

    public DifferentialDriveWheelPositions getMeasuredWheelPositions() {
        return new DifferentialDriveWheelPositions(this.LeftLeadMotor.getEncoder().getPosition(),
                this.RightLeadMotor.getEncoder().getPosition());
    }

    public ChassisSpeeds getMeasuredRobotRelativeChassisSpeeds() {
        return Kinematics.toChassisSpeeds(this.getMeasuredWheelSpeeds());
    }

    public void resetPose(Pose2d pose) {
        this.PoseEstimator.resetPose(pose);
        this.PosePublisher.accept(pose);
    }

    public void driveRobotRelative(ChassisSpeeds desiredSpeeds) {
        // https://docs.revrobotics.com/revlib/spark/closed-loop/velocity-control-mode

        var wheelSpeeds = Kinematics.toWheelSpeeds(desiredSpeeds);

        var portRPM = wheelSpeeds.leftMetersPerSecond / Constants.Drive.DISTANCE_PER_MOTOR_REV.in(Meters) * 60;
        var starboardRPM = wheelSpeeds.rightMetersPerSecond / Constants.Drive.DISTANCE_PER_MOTOR_REV.in(Meters) * 60;

        this.LeftController.setReference(portRPM, SparkBase.ControlType.kVelocity);
        this.RightController.setReference(starboardRPM, SparkBase.ControlType.kVelocity);
    }

    @Override
    public void periodic() {

        this.PoseEstimator.update(this.getGyroRotation(), this.getMeasuredWheelPositions());

        var poseEstimation = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.Limelight.Name);
        if (poseEstimation != null && poseEstimation.tagCount > 1) {
            this.PoseEstimator.addVisionMeasurement(poseEstimation.pose, poseEstimation.timestampSeconds);
        }

        var updatedPose = this.PoseEstimator.getEstimatedPosition();

        PosePublisher.accept(updatedPose);

        if (DriverStation.isDisabled())
            this.stop();
    }

    public void stop() {
        this.LeftController.setReference(0, SparkBase.ControlType.kVelocity);
        this.RightController.setReference(0, SparkBase.ControlType.kVelocity);
        this.LeftLeadMotor.stopMotor();
        this.RightLeadMotor.stopMotor();
        this.LeftFollowMotor.stopMotor();
        this.RightFollowMotor.stopMotor();
    }
}