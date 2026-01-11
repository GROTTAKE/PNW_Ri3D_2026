package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {

    private final SparkMax pullMotor;
    private final RelativeEncoder encoder;

    public ClimbSubsystem()
    {
        this.pullMotor = new SparkMax(Constants.Climber.CAN_ID, MotorType.kBrushless);
        
        var pullConfig = new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(75);
        
        this.pullMotor.configure(pullConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        this.encoder = this.pullMotor.getEncoder();
        
        this.encoder.setPosition(0);
    }

    public void setVoltage(double volts)
    {
        this.pullMotor.setVoltage(volts);
    }

    public Command pullCmd()
    {
        // I believe the travel is 9.5 inches from extended to the bottom.

        return Commands
            .runEnd(() -> this.pullMotor.setVoltage(Constants.Climber.PULL_VOLTAGE), () -> this.pullMotor.stopMotor(), this)
            .onlyWhile(() -> getVerticalDistance().lte(Inches.of(9))); // Go until it travels 9 inches
    }

    public Distance getVerticalDistance()
    {
        return Inches.of(Constants.Climber.SPROCKET_CIRCUMFERENCE.times(this.encoder.getPosition() / Constants.Climber.GEAR_RATIO).abs(Inches));
    }

    @Override
    public void periodic() 
    {
        SmartDashboard.putNumber("Climber/VerticalDistance", getVerticalDistance().in(Inches));

        if (DriverStation.isTest())
        {
            SmartDashboard.putNumber("Climber/Amps", this.pullMotor.getOutputCurrent());
        }

        if (DriverStation.isDisabled()) 
        {
            this.pullMotor.stopMotor();
        }
    }

    public void stop()
    {
        this.pullMotor.stopMotor();
    }
}
