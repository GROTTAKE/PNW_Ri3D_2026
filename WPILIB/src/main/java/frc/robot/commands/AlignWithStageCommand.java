package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controlLoops.CircularStageController;
import frc.robot.models.CircularStage;
import frc.robot.subsystems.DriveSubsystem;

public class AlignWithStageCommand extends Command 
{
    public final DriveSubsystem Drive;
    public final CircularStage Stage;
    private final CircularStageController Controller;

    public AlignWithStageCommand(DriveSubsystem drive, CircularStage targetStage, Angle angleTolerance, AngularVelocity rotVelocity, AngularAcceleration rotAccel, LinearVelocity translateConstraint)
    {
        this.Drive = drive;
        this.Stage = targetStage;
        this.Controller = new CircularStageController(targetStage, rotVelocity, rotAccel, translateConstraint, angleTolerance);

        this.addRequirements(drive);
    }

    @Override
    public void execute() {
       
        var alignSpeeds = this.Controller.execute(this.Drive.getEstimatedPose());
        this.Drive.driveRobotRelative(alignSpeeds);

    }

    @Override
    public boolean isFinished() {
       
        return this.Controller.atGoal(this.Drive.getEstimatedPose());
    }

    @Override
    public void end(boolean interrupted) {
        
        this.Drive.Stop();
    }
}
