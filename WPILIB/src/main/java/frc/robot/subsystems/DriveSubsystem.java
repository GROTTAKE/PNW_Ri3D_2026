package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.LimelightHelpers;

public class DriveSubsystem extends RobotSubsystem<Robot>
{
    public DifferentialDrivetrainSim SimulatedDrive;

    private SparkMax LeftLeadMotor, RightLeadMotor, LeftFollowMotor, RightFollowMotor;
    
    private final DifferentialDriveKinematics Kinematics = new DifferentialDriveKinematics(Constants.Drive.TRACK_WIDTH);
    private final SimpleMotorFeedforward FeedForward = new SimpleMotorFeedforward(0.15, 2.5, 0.53);
    private final SimpleMotorFeedforward RotationFeedForward = new SimpleMotorFeedforward(0.65, 1, 0);

    private AHRS NavX;
    private DifferentialDrivePoseEstimator PoseEstimator;

    private ChassisSpeeds Speeds = new ChassisSpeeds();

    public DriveSubsystem(Robot robot)
    {
        super(robot);
    }

    protected void initializeReal()
    {
        this.LeftLeadMotor = new SparkMax(Constants.Drive.LEFT_LEAD_CAN_ID, MotorType.kBrushless);
        this.LeftFollowMotor = new SparkMax(Constants.Drive.LEFT_FOLLOW_CAN_ID, MotorType.kBrushless);
        this.RightFollowMotor = new SparkMax(Constants.Drive.RIGHT_FOLLOW_CAN_ID, MotorType.kBrushless);
        this.RightLeadMotor = new SparkMax(Constants.Drive.RIGHT_LEAD_CAN_ID, MotorType.kBrushless);

        this.LeftLeadMotor.configure(new SparkMaxConfig().openLoopRampRate(0.5), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        this.RightLeadMotor.configure(new SparkMaxConfig().openLoopRampRate(0.5), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        this.LeftFollowMotor.configure(new SparkMaxConfig().follow(this.LeftLeadMotor), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        this.LeftFollowMotor.configure(new SparkMaxConfig().follow(this.LeftLeadMotor), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        this.LeftLeadMotor.getEncoder().setPosition(0);
        this.RightLeadMotor.getEncoder().setPosition(0);

        this.NavX = new AHRS(AHRS.NavXComType.kMXP_SPI);
        this.PoseEstimator = new DifferentialDrivePoseEstimator(Kinematics, this.getGyroRotation(), 0, 0, new Pose2d());
    }

    @Override
    protected void initializeSimulated()
    {
        // this.SimulatedDrive = new DifferentialDrivetrainSim(
        //     DCMotor.getNEO(2),
        //     Constants.Drive.GEAR_RATIO,
        //     8,
        //     edu.wpi.first.math.util.Units.lbsToKilograms(90),
        //     Constants.Drive.WHEEL_DIAMETER.div(2).in(Meters),
        //     Constants.Drive.TRACK_WIDTH.in(Meters),
        //     null
        //     // VecBuilder.fill(0.001, 0.001, 0.001, 0.05, 0.05, 0.005, 0.005)
        // );

        // var startingPosition = FieldConstants.GetStartingPosition();
        // if (startingPosition.isPresent())
        // {
        //     this.SimulatedDrive.setPose(startingPosition.get());
        // }
    }

    public DifferentialDriveWheelPositions GetWheelPositions()
    {
        return new DifferentialDriveWheelPositions(
            RobotBase.isReal() ? this.GetLeftWheelPosition() : this.SimulatedDrive.getLeftPositionMeters(),
            RobotBase.isReal() ? this.GetRightWheelPosition() : this.SimulatedDrive.getRightPositionMeters());
    }

    /**
     * @return Left wheel velocity in M/s
     */
    public double GetLeftWheelVelocity()
    {
        return (this.LeftLeadMotor.getEncoder().getVelocity() / 60) / Constants.Drive.GEAR_RATIO * 2 * Math.PI * Constants.Drive.WHEEL_RADIUS.in(Meters);
    }

    /**
     * @return Right wheel velocity in M/s
     */
    public double GetRightWheelVelocity()
    {
        return (this.RightLeadMotor.getEncoder().getVelocity() / 60) / Constants.Drive.GEAR_RATIO * 2 * Math.PI * Constants.Drive.WHEEL_RADIUS.in(Meters);
    }

    /**
     * @return Left wheel velocity in M/s
     */
    public double GetLeftWheelPosition()
    {
        return this.LeftLeadMotor.getEncoder().getPosition() / Constants.Drive.GEAR_RATIO * 2 * Math.PI * Constants.Drive.WHEEL_RADIUS.in(Meters);
    }

    /**
     * @return Right wheel velocity in M/s
     */
    public double GetRightWheelPosition()
    {
        return this.RightLeadMotor.getEncoder().getPosition() / Constants.Drive.GEAR_RATIO * 2 * Math.PI * Constants.Drive.WHEEL_RADIUS.in(Meters);
    }

    public void ResetEncoders()
    {
        if (RobotBase.isReal())
        {
            this.LeftLeadMotor.getEncoder().setPosition(0);
            this.RightLeadMotor.getEncoder().setPosition(0);
        } 
        else
        {
            this.SimulatedDrive.setPose(new Pose2d());
        }
    }

    /**
     * Stops all motors and resets the target speed to zero.
     */
    public void Stop()
    {
        // Bypass Set()
        this.Speeds = new ChassisSpeeds(0, 0, 0);

        if (RobotBase.isReal())
        {
            this.LeftLeadMotor.stopMotor();
            this.RightLeadMotor.stopMotor();
            // Just in case, stop following motors.
            this.LeftFollowMotor.stopMotor();
            this.RightFollowMotor.stopMotor();
        } 
        else
        {
            this.SimulatedDrive.setInputs(0, 0);
        }
    }

    public ChassisSpeeds GetWheelSpeeds()
    {
        return this.Kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(
            RobotBase.isReal() ? this.GetLeftWheelVelocity() : this.SimulatedDrive.getLeftVelocityMetersPerSecond(),
            RobotBase.isReal() ? this.GetRightWheelVelocity() : this.SimulatedDrive.getRightVelocityMetersPerSecond()));
    }

    public ChassisSpeeds GetSpeeds() 
    {
        return this.Speeds;
    }

    public Rotation2d getGyroRotation()
    {
        return RobotBase.isSimulation() ? this.SimulatedDrive.getPose().getRotation() : this.NavX.getRotation2d();
    }

    /**
     * Sets the Drivetrain to move at the specified rate.
     * 
     * @return The clamped or rate-limited speeds.
     */
    public void driveRobotRelative(ChassisSpeeds speeds)
    {
        this.Speeds = speeds;
        this.UpdateMotors();
    }

    // public void SetArcade(double xSpeed, double zRotation)
    // {
    //     xSpeed = MathUtil.applyDeadband(xSpeed, 0.1);
    //     xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);

    //     zRotation = MathUtil.applyDeadband(zRotation, 0.1);
    //     zRotation = Math.copySign(zRotation * zRotation, zRotation);

    //     this.Set(new ChassisSpeeds(
    //         xSpeed * Constants.Drivetrain.MaxXSpeed,
    //         0,
    //         Constants.Drivetrain.MaxAngularSpeed.times(zRotation).getRadians()
    //     ));
    // }

    // public void SetIdle(IdleMode mode)
    // {
    //     if (RobotBase.isReal())
    //     {
    //         this.RightFollowMotor.setIdleMode(mode);
    //         this.RightLeadMotor.setIdleMode(mode);
    //         this.LeftFollowMotor.setIdleMode(mode);
    //         this.LeftLeadMotor.setIdleMode(mode);
    //     }
    // }

    protected void UpdateMotors()
    {
        if (DriverStation.isDisabled())
        {
            this.Stop();
            return;
        }

        double omegaRadiansPerSecond = this.RotationFeedForward.calculate(-this.Speeds.omegaRadiansPerSecond);

        var wheelSpeeds = this.Kinematics.toWheelSpeeds(new ChassisSpeeds(
            Math.min(Math.max(this.Speeds.vxMetersPerSecond, -Constants.Drive.MAX_TRANSLATION_VELOCITY.in(MetersPerSecond)), Constants.Drive.MAX_TRANSLATION_VELOCITY.in(MetersPerSecond)),
            0,
            Math.min(Math.max(omegaRadiansPerSecond, -Constants.Drive.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond)), Constants.Drive.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond))));

        if (RobotBase.isReal())
        {
            this.LeftLeadMotor.setVoltage(FeedForward.calculate(wheelSpeeds.leftMetersPerSecond, 0.05));
            this.RightLeadMotor.setVoltage(FeedForward.calculate(wheelSpeeds.rightMetersPerSecond, 0.05));
        } 
        else
        {
            this.SimulatedDrive.setInputs(
                FeedForward.calculate(this.SimulatedDrive.getLeftVelocityMetersPerSecond(), wheelSpeeds.leftMetersPerSecond),
                FeedForward.calculate(this.SimulatedDrive.getRightVelocityMetersPerSecond(), wheelSpeeds.rightMetersPerSecond));
        }
    }

    @Override
    public void periodic()
    {
        if (!DriverStation.isDisabled())
        {
            this.UpdateMotors();
        }

        this.PoseEstimator.update(getGyroRotation(), this.GetWheelPositions());

        var llPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.Limelight.Name);
        if (llPose != null && llPose.pose != null && llPose.tagCount >= 2)
        {
            // Once ll has spotted our position ensure two april tags are seen for megatag.
            this.PoseEstimator.addVisionMeasurement(llPose.pose, llPose.timestampSeconds);
        }
        
    }

    public Pose2d getEstimatedPose()
    {
        return this.PoseEstimator.getEstimatedPosition();
    }

    @Override
    public void simulationPeriodic()
    {
        this.SimulatedDrive.update(0.02);
    }

    @Override
    public void initSendable(SendableBuilder builder)
    {
        builder.addDoubleProperty("Desired Speed", () -> this.Speeds.vxMetersPerSecond, null);
        builder.addDoubleProperty("Desired Rotation", () -> edu.wpi.first.math.util.Units.radiansToDegrees(this.Speeds.omegaRadiansPerSecond),
                null);

        builder.addDoubleProperty("Actual Speed", () ->this.GetWheelSpeeds().vxMetersPerSecond, null);
        builder.addDoubleProperty("Actual Rotation",
                () -> -edu.wpi.first.math.util.Units.radiansToDegrees(this.GetWheelSpeeds().omegaRadiansPerSecond), null);

        builder.addStringProperty("Blocking Command", () ->
        {
            var requiredCommand = CommandScheduler.getInstance().requiring(this);
            boolean isDefault = requiredCommand == this.getDefaultCommand();

            return requiredCommand != null && !isDefault ? requiredCommand.getName() : "None";
        }, null);

    }
}