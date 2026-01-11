package frc.robot.models;

import com.revrobotics.spark.config.SparkBaseConfig;

public record PIDValues(double P, double I, double D)
{
    public SparkBaseConfig applyToSparkClosedLoopConfig(SparkBaseConfig config)
    {
        config.closedLoop.pid(this.P, this.I, this.D);

        return config;
    }
}