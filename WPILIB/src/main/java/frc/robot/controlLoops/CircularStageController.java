package frc.robot.controlLoops;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.models.CircularStage;

public class CircularStageController 
{
    public final CircularStage Stage;
    
    private final LinearVelocity TranslateSpeed;
    private final FaceAtController FaceAtController;

    public CircularStageController(CircularStage stage, AngularVelocity rotVelocity, AngularAcceleration rotAccel, LinearVelocity translateConstraint, Angle angleTolerance)
    {
        this.Stage = stage;
        this.TranslateSpeed = translateConstraint;
        this.FaceAtController = new FaceAtController(this.Stage.Point, new Constraints(rotVelocity.in(DegreesPerSecond), rotAccel.in(DegreesPerSecondPerSecond)), angleTolerance);
    }

    public boolean atGoal(Pose2d rPose)
    {
        return this.FaceAtController.atGoal() && this.getDirectionSign(rPose) == 0;
    }

    private Distance getDistanceFromPoint(Pose2d rPose)
    {
        return Meters.of(this.Stage.Point.getDistance(rPose.getTranslation()));
    }

    /**
     * Returns 1, 0, or -1 depending on if the Robot needs to go forward, not move, or go backward.
     */
    private int getDirectionSign(Pose2d rPose)
    {
        var distanceFromPoint = getDistanceFromPoint(rPose);

        if (distanceFromPoint.lte(Constants.Launcher.MIN_SHOOTING_DISTANCE))
        {
            return -1;
        }
        else if (distanceFromPoint.gte(Constants.Launcher.MAX_SHOOTING_DISTANCE))
        {
            return 1;
        }
        else return 0;
    }

    private static final Distance DISTANCE_DEADBAND = Meters.of(0.05); // 5 cm
    private static final double KP_TRANSLATE = 1.5; // tune


    public ChassisSpeeds execute(Pose2d rPose)
    {
        // Always finish rotation first
        if (!FaceAtController.atGoal())
        {
            return new ChassisSpeeds(0, 0, FaceAtController.execute(rPose).getRadians());
        }

        var distance = getDistanceFromPoint(rPose);

        // Compute signed error relative to shooting band
        double errorMeters = 0.0;

        if (distance.gt(Constants.Launcher.MAX_SHOOTING_DISTANCE))
        {
            errorMeters =
                distance.minus(Constants.Launcher.MAX_SHOOTING_DISTANCE).in(Meters);
        }
        else if (distance.lt(Constants.Launcher.MIN_SHOOTING_DISTANCE))
        {
            errorMeters =
                distance.minus(Constants.Launcher.MIN_SHOOTING_DISTANCE).in(Meters);
        }

        // Deadband → no oscillation
        if (Math.abs(errorMeters) < DISTANCE_DEADBAND.in(Meters))
        {
            return new ChassisSpeeds();
        }

        // Proportional speed (clamped)
        double vx =
            Math.copySign(
                Math.min(
                    Math.abs(errorMeters) * KP_TRANSLATE,
                    TranslateSpeed.in(MetersPerSecond)),
                errorMeters
            );

        return new ChassisSpeeds(vx, 0, 0);
    }

}