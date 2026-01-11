package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;

import java.time.Duration;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

public class RumbleCommand extends Command
{
    public final XboxController controller;
    private final double amount;

    public RumbleCommand(XboxController controller, double amount)
    {
        this.controller = controller;
        this.amount = amount;
    }

    @Override
    public void execute() {
        
        controller.setRumble(RumbleType.kBothRumble, this.amount);

    }

    @Override
    public void end(boolean interrupted) {
        
        controller.setRumble(RumbleType.kBothRumble, 0);

    }
}
