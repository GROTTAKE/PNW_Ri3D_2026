package frc.robot.controllers;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.time.Duration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.RumbleCommand;

public class OperatorController 
{
    public final XboxController Base;
    
    public OperatorController(XboxController controller)
    {
        this.Base = controller;
    }
    
    public ChassisSpeeds getArcadeChassisSpeeds()
    {
        return new ChassisSpeeds(
            MathUtil.applyDeadband(-Base.getLeftY(), 0.1) * Constants.Drive.MAX_TRANSLATION_VELOCITY.in(MetersPerSecond),
            0,
            MathUtil.applyDeadband(Base.getRightX(), 0.1) * Constants.Drive.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond)
        );
    }

    /**
     * Aligns to hub stage and automatically launches fuel.
     */
    public boolean getAutoLaunchButton()
    {
        return this.getLeftTriggerDead() > 0.5;
    }

    /**
     * 
     * @return 0, if not triggered, [0-1) if triggered.
     */
    public double getManualLaunchButton()
    {
        if (this.Base.getPOV() == 0)
        {
            return 0.25;
        }
        else if (this.Base.getPOV() == 90)
        {
            return 0.50;
        }
        else if (this.Base.getPOV() == 180)
        {
            return 0.75;
        }
        else if (this.Base.getPOV() == 270)
        {
            return 1;
        }
        else
        {
            return Math.abs(this.getRightTriggerDead());
        }
    }

    public boolean getIntakeButton()
    {
        return this.Base.getLeftBumperButton();
    }

    public boolean getOuttakeButton()
    {
        return this.Base.getRightBumperButton();
    }

    public double getLeftTriggerDead()
    {
        return MathUtil.applyDeadband(this.Base.getLeftTriggerAxis(), 0.1);
    }

    public double getRightTriggerDead()
    {
        return MathUtil.applyDeadband(this.Base.getRightTriggerAxis(), 0.1);
    }

    public Command rumbleCmd(double amount, Duration duration)
    {
        return new RumbleCommand(Base, amount).withTimeout(duration.toSeconds());
    }
}
