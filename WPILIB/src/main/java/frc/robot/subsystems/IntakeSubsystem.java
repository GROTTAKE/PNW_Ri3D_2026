package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    public SparkMax Motor;

    public IntakeSubsystem()
    {
        super();

        this.Motor = new SparkMax(Constants.Intake.CAN_ID, MotorType.kBrushless);
    }

    public Command intakeCmd()
    {
        return Commands.runEnd(this::intake, this::stop, this);   
    }

    public void intake()
    {
        this.Motor.set(0.30);
    }

    public Command outtakeCmd()
    {
        return Commands.runEnd(this::outtake, this::stop, this);   
    }

    public void outtake()
    {
        this.Motor.set(-0.30);
    }
    
    public void stop()
    {
        this.Motor.stopMotor();
    }

    @Override
    public void periodic() {
        
        if (DriverStation.isDisabled())
        {
            this.stop();
            return;
        }

    }
    
}
