package frc.robot;

import frc.robot.*;
import frc.robot.models.CircularStage;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Game 
{
    public static Translation2d getAllianceHub()
    {
        return DriverStation.getAlliance().get() == Alliance.Blue ? Constants.Field_BLUE.BLUE_HUB : Constants.Field_BLUE.BLUE_HUB;
    }

    public static CircularStage getAllianceHubStage()
    {
        return DriverStation.getAlliance().get() == Alliance.Blue ? Constants.Launcher.BLUE_HUB_STAGE : Constants.Launcher.RED_HUB_STAGE;
    }

    // public static Distance distanceFromAllianceHub(Robot robot)
    // {        
    //     return Meters.of(getAllianceHub().getDistance(robot.Drive.getPo().getTranslation()));
    // }
}
