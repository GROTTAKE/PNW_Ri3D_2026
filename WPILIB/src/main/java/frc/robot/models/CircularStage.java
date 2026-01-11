package frc.robot.models;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class CircularStage 
{
    public final Translation2d Point;
    public final Distance Min, Max;    

    public CircularStage(Translation2d point, Distance min, Distance max)
    {
        this.Point = point;
        this.Min = min;
        this.Max = max;
    }
}
