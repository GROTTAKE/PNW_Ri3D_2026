package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase
{
    public SparkMax Motor;
    public SparkClosedLoopController MotorController; 

    // private final StructPublisher<Double> MeasuredRPMPublisher = 

    public LauncherSubsystem()
    {
        this.Motor = new SparkMax(Constants.Launcher.CAN_ID, MotorType.kBrushless);
        this.MotorController = this.Motor.getClosedLoopController();
    }

    public void setRPM(int RPM)
    {
        this.MotorController.setReference(RPM, ControlType.kVelocity);
    }

    public double getMeasuredRPM()
    {
        return this.Motor.getEncoder().getVelocity();
    }

    @Override
    public void periodic() {
        
        if (DriverStation.isDisabled())
        {
            this.stop();
        }
        
    }

    public void stop()
    {
        this.MotorController.setReference(0, ControlType.kVelocity);
        this.Motor.stopMotor();
    }
}
