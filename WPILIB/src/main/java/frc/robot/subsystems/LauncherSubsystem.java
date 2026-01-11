package frc.robot.subsystems;

import frc.robot.*;
import frc.robot.controlLoops.JammyJim;
import frc.robot.controlLoops.JammyJim.JammyResults;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.models.PIDValues;
import frc.robot.utils.ElapsedTimer;

public class LauncherSubsystem extends SubsystemBase
{
    private final SparkMax launchMotor, feederMotor;
    private final SparkClosedLoopController launchMotorController;

    public boolean doFeed;

    public AngularVelocity desiredRPM;
    public double desiredVoltage;
    public LaunchControlMode controlMode;

    private PIDValues launchPID;

    private final LinearFilter launchMotorAmpsAvg;
    private final JammyJim<Double> launchJammyJim;

    private final LinearFilter feedMotorAmpsAvg;
    private final JammyJim<Double> feedJammyJim;


    public boolean isLauncherJammed;
    public boolean isFeederJammed;

    public LauncherSubsystem()
    {
        super();

        this.desiredRPM = DegreesPerSecond.of(0);
        this.desiredVoltage = 0;
        this.controlMode = LaunchControlMode.RPM;
        this.doFeed = false;
        this.isLauncherJammed = false;

        this.launchMotor = new SparkMax(Constants.Launcher.LAUNCH_CAN_ID, MotorType.kBrushless);
        this.feederMotor = new SparkMax(Constants.Launcher.FEEDER_CAN_ID, MotorType.kBrushless);

        this.launchPID = new PIDValues(Constants.Launcher.P, Constants.Launcher.I, Constants.Launcher.D);
        
        var launchConfig = makeLaunchMotorConfig(launchPID);
        var feedConfig = new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(80)
            .inverted(true)
            .disableFollowerMode();

        this.launchMotor.configure(launchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.feederMotor.configure(feedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // https://docs.revrobotics.com/revlib/spark/closed-loop/maxmotion-velocity-control
        this.launchMotorController = this.launchMotor.getClosedLoopController();

        this.launchMotorAmpsAvg = LinearFilter.movingAverage((int)((1 / Robot.kDefaultPeriod) / 1)); // 1 second.
        this.launchJammyJim = new JammyJim<Double>(75d, Constants.Launcher.JAM_UNDO_DURATION);

        this.feedMotorAmpsAvg = LinearFilter.movingAverage((int)((1 / Robot.kDefaultPeriod) / 1)); // 1 second.
        this.feedJammyJim = new JammyJim<Double>(75d, Constants.Launcher.JAM_UNDO_DURATION);
        
        if (DriverStation.isTest())
        {
            SmartDashboard.putNumber("Launcher/PID/P", this.launchPID.P());
            SmartDashboard.putNumber("Launcher/PID/I", this.launchPID.I());
            SmartDashboard.putNumber("Launcher/PID/D", this.launchPID.D());
        }
    }

    public Command launchAtHubCmd(frc.robot.subsystems.DriveSubsystem drive, Translation2d hubLocation)
    {
        return Commands.runEnd(() -> {

            var distanceFromAllianceHub = Meters.of(drive.getEstimatedPose().getTranslation().getDistance(hubLocation));
            var calculatedRPM = Constants.Launcher.calculateLaunchRPM(distanceFromAllianceHub).get();

            this.setLaunchVelocity(calculatedRPM);
            this.doFeed = true;

        }, () -> this.stop(), this);
    }

    // public void setDesiredLaunchVelocity(AngularVelocity velocity)
    // {
    //     this.desiredRPM = RPM.of(Math.min(velocity.in(RPM), Constants.Launcher.MAX_RPM.in(RPM)));
    // }

    public AngularVelocity getMeasuredRPM()
    {
        return RPM.of(this.launchMotor.getEncoder().getVelocity());
    }

    public AngularVelocity getDesiredRPM()
    {
        return this.desiredRPM;
    }

    private AngularVelocity getDeltaRPM()
    {
        return this.getDesiredRPM().minus(this.getMeasuredRPM());
    }

    public boolean isReadyToLaunch()
    {
        if (controlMode == LaunchControlMode.Voltage) return true;

        // TODO: Once we do the RPM v Distance tuning refine the allowed RPM tolerance.

        var deltaRPM = this.getDeltaRPM();
        
        var toleranceRPM = RPM.of(100); // Tolerance is 5% of desired RPM.

        return deltaRPM.abs(RotationsPerSecond) <= toleranceRPM.in(RotationsPerSecond);
    }

    @Override
    public void periodic() {

        if (DriverStation.isTest())
        {
            // Take in PID values.

            var dashPID = new PIDValues(
                SmartDashboard.getNumber("Launcher/PID/P", Constants.Launcher.P), 
                SmartDashboard.getNumber("Launcher/PID/I", Constants.Launcher.I),
                SmartDashboard.getNumber("Launcher/PID/D", Constants.Launcher.D));

            if (!dashPID.equals(this.launchPID))
            {
                // Changed.
                this.launchPID = dashPID;
                this.launchMotor.configure(makeLaunchMotorConfig(this.launchPID), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

                System.out.println("TEST: Launcher PID Updated!");

                SmartDashboard.putString("Launcher/PID/Updated", this.launchPID.toString());
                SmartDashboard.putNumber("Launcher/PID/P", this.launchPID.P());
                SmartDashboard.putNumber("Launcher/PID/I", this.launchPID.I());
                SmartDashboard.putNumber("Launcher/PID/D", this.launchPID.D());
            }
        }

        var launchMotorAmps = this.launchMotor.getOutputCurrent();
        var avgLaunchMotorAmps = this.launchMotorAmpsAvg.calculate(launchMotorAmps);

        var feedMotorAmps = this.feederMotor.getOutputCurrent();
        var avgFeedMotorAmps = this.feedMotorAmpsAvg.calculate(feedMotorAmps);

        SmartDashboard.putBoolean("Launcher/IsReady", isReadyToLaunch());
        SmartDashboard.putString("Launcher/ControlMode", controlMode.toString());
        SmartDashboard.putNumber("Launcher/DesiredRPM", getDesiredRPM().in(RPM));
        SmartDashboard.putNumber("Launcher/MeasuredRPM", getMeasuredRPM().in(RPM));
        SmartDashboard.putNumber("Launcher/LaunchAmps", launchMotorAmps);
        SmartDashboard.putNumber("Launcher/LaunchAvgAmps", avgLaunchMotorAmps);
        SmartDashboard.putNumber("Launcher/FeedAmps", feedMotorAmps);
        SmartDashboard.putNumber("Launcher/FeedAvgAmps", avgFeedMotorAmps);
        SmartDashboard.putNumber("Launcher/%", this.launchMotor.get() * 100);

        if (DriverStation.isDisabled())
        {
            this.stop();
            return;
        }

        // Launch Jamming Detection

        var launchJam = this.launchJammyJim.update(avgLaunchMotorAmps);
        this.isLauncherJammed = launchJam == JammyResults.Unjamming;

        if (launchJam == JammyResults.Unjammed)
        {
            System.out.println("WOOOOO UNJAMMED?");
        }

        SmartDashboard.putString("Launcher/LaunchJam", launchJam.toString());

        if (this.isLauncherJammed)
        {
            this.launchMotor.setVoltage(-12);
            return;
        }

        // Feed Jamming Detection

        var feedJam = this.feedJammyJim.update(avgFeedMotorAmps);
        this.isFeederJammed = feedJam == JammyResults.Unjamming;

        SmartDashboard.putString("Launcher/FeedJam", feedJam.toString());

        if (this.isFeederJammed)
        {
            this.feederMotor.setVoltage(-12);
            return;
        }

        // Handle control mode, and set desired RPM or Voltage.
        
        if (controlMode == LaunchControlMode.RPM)
        {
            this.setLaunchVelocity(desiredRPM);
        }
        else if (controlMode == LaunchControlMode.Voltage)
        {
            this.launchMotor.setVoltage(desiredVoltage);
        }

        if (doFeed && isReadyToLaunch())
        {
            this.feederMotor.set(0.40);
        }
        else
        {
            this.feederMotor.stopMotor();
        }
    }

    private void setLaunchVelocity(AngularVelocity velocity)
    {
        this.launchMotorController.setReference(velocity.in(RPM), ControlType.kMAXMotionVelocityControl);
    }

    public void stop()
    {
        this.desiredRPM = RPM.zero();
        this.desiredVoltage = 0;
        this.doFeed = false;
        this.setLaunchVelocity(RPM.zero());
        this.launchMotor.stopMotor();
        this.feederMotor.stopMotor();
    }

    private static SparkBaseConfig makeLaunchMotorConfig(PIDValues launchPID)
    {
        var launchConfig = new SparkMaxConfig()
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(80)
            .disableFollowerMode();            

        launchConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);

        // launchConfig.closedLoop.velocityFF(1 / Constants.Launcher.LAUNCHER_FREE_SPINNING_VELOCITY.in(RPM)).pid(0.00105, 0, 0).maxMotion.maxAcceleration(Constants.Launcher.LAUNCHER_MAX_RAMP_VELOCITY.in(RPM));
        // launchConfig.closedLoop.velocityFF(1 / Constants.Launcher.LAUNCHER_FREE_SPINNING_VELOCITY.in(RPM)).pid(0.00063, 0.0038, 0.00096).maxMotion.maxAcceleration(Constants.Launcher.LAUNCHER_MAX_RAMP_VELOCITY.in(RPM));
        launchConfig.closedLoop
            .velocityFF(1 / Constants.Launcher.LAUNCHER_FREE_SPINNING_VELOCITY.in(RPM))
            .maxMotion
                .allowedClosedLoopError(0.083 * 2)
                .maxAcceleration(Constants.Launcher.LAUNCHER_MAX_RAMP_VELOCITY.in(RPM));

        launchPID.applyToSparkClosedLoopConfig(launchConfig);

        return launchConfig;
    }

    public static enum LaunchControlMode
    {
        RPM,
        Voltage
    }

}
