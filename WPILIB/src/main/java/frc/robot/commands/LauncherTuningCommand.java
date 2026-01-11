package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import java.time.Duration;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.utils.ElapsedTimer;

public class LauncherTuningCommand extends Command {

    private AngularVelocity Current, Max, Interval;
    private Optional<ElapsedTimer> StateTimer;
    private final LauncherSubsystem Launcher;
    private final Supplier<Boolean> DoNextLevel, DoPrevLevel, DoShoot;

    public LauncherTuningCommand(

        LauncherSubsystem launcher,
        AngularVelocity min,
        AngularVelocity max,
        AngularVelocity interval,
        Supplier<Boolean> doNextLevel,
        Supplier<Boolean> doPrevLevel,
        Supplier<Boolean> doShoot

    )
    {
        this.Current = min;
        this.Max = max;
        this.Interval = interval;
        this.Launcher = launcher;
        this.DoNextLevel = doNextLevel;
        this.DoPrevLevel = doPrevLevel;
        this.DoShoot = doShoot;
        this.StateTimer = Optional.empty();

        this.addRequirements(launcher);
    }

    @Override
    public void execute()
    {

        SmartDashboard.putNumber("LauncherTuningCommand/CurrentRPM", Current.in(RPM));
        SmartDashboard.putNumber("LauncherTuningCommand/IntervalRPM", Interval.in(RPM));

        if (StateTimer.isPresent() && !StateTimer.get().hasElapsed())
        {
            return;   
        }

        if (StateTimer.isPresent() && StateTimer.get().hasElapsed())
        {
            this.StateTimer = Optional.empty();
            this.Launcher.stop();
        }

        if (DoShoot.get())
        {
            this.StateTimer = Optional.of(new ElapsedTimer(Constants.Launcher.LAUNCHER_RAMP_FULL_SECONDS.plus(Duration.ofSeconds(1))));
            this.StateTimer.get().Timer.start();
            this.Launcher.desiredRPM = Current;
            this.Launcher.doFeed = true;
        }
        else if (DoNextLevel.get())
        {
            this.Current = this.Current.plus(this.Interval);
        }
        else if (DoPrevLevel.get())
        {
            this.Current = this.Current.minus(this.Interval);
        }

    }

    @Override
    public boolean isFinished() {
        
        return Current.gte(Max);

    }

    @Override
    public void end(boolean interrupted) {
        
        this.Launcher.stop();

    }
    
}
