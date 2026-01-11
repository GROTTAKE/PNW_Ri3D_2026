package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Game;
import frc.robot.subsystems.*;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class GameCommands {
    
    public static final Command autoStageAndLaunchFuelInAllianceHub(frc.robot.Robot robot)
    {
        Command alignCommand = new AlignWithStageCommand(
            robot.Drive, 
            Game.getAllianceHubStage(), 
            Degrees.of(10),
            Constants.Drive.MAX_ANGULAR_VELOCITY, 
            Constants.Drive.MAX_ANGULAR_ACCEL, 
            Constants.Drive.MAX_TRANSLATION_VELOCITY); 

        Command alignThenLaunchCommand = alignCommand.andThen(robot.Launcher.launchAtHubCmd(robot.Drive, Game.getAllianceHub()));

        return alignThenLaunchCommand;
    }

    public static final Command stageAllianceHubCmd(frc.robot.Robot robot)
    {
        // var currentAlliance = DriverStation.getAlliance().get();

        // AlignWithStageCommand alignCommand = new AlignWithStageCommand(
        //     robot.Drive, 
        //     currentAlliance == Alliance.Blue ? Constants.Launcher.BLUE_HUB_STAGE : Constants.Launcher.RED_HUB_STAGE, 
        //     Degrees.of(10),
        //     Constants.Drive.MAX_ANGULAR_VELOCITY, 
        //     Constants.Drive.MAX_ANGULAR_ACCEL, 
        //     Constants.Drive.MAX_TRANSLATION_VELOCITY); 

        // return alignCommand;

        throw new UnsupportedOperationException();
    }

}
