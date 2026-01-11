package frc.robot.controlLoops;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;

public class FaceAtController 
{
    public final Translation2d FaceTo;
    private final PIDController PID;

    public FaceAtController(Translation2d faceTo, TrapezoidProfile.Constraints rotConstraints, Angle angleTolerance)
    {
        this.FaceTo = faceTo;
        this.PID = new PIDController(3.2, 0, 0.000005);
        this.PID.setTolerance(angleTolerance.in(Degrees));
        this.PID.enableContinuousInput(-180, 180);
    }

    public boolean atGoal()
    {
        return this.PID.atSetpoint();
    }

    public Rotation2d getRequiredHeading(Pose2d rPose)
	{
		return FaceTo.minus(rPose.getTranslation()).getAngle();
	}

    public Rotation2d execute(Pose2d rPose)
    {
        return Rotation2d.fromDegrees(this.PID.calculate(rPose.getRotation().getDegrees(), this.getRequiredHeading(rPose).getDegrees()));
    }
}
