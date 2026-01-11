// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import java.time.Duration;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.GameCommands;
import frc.robot.commands.LauncherTuningCommand;
import frc.robot.controllers.OperatorController;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LauncherSubsystem.LaunchControlMode;

public class Robot extends TimedRobot {

    private OperatorController Controller;

    // Subsystem

    public final DriveSubsystem Drive;
    public final LauncherSubsystem Launcher;
    public final IntakeSubsystem Intake;
    // public final ClimbSubsystem Climber;

    /** Called once at the beginning of the robot program. */
    public Robot() {

        this.Controller = new OperatorController(new XboxController(0));
        this.Drive = new DriveSubsystem(this);

        this.Launcher = new LauncherSubsystem();

        this.Intake = new IntakeSubsystem();

        // this.Climber = new ClimbSubsystem();

    }

    @Override
    public void robotInit() {

        System.out.println("Welcome Team 72, Ri3D @ PNW");
                
    }

    @Override
    public void disabledInit() 
    {
        this.Drive.Stop();    
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void robotPeriodic() {

        CommandScheduler.getInstance().run();

        if (this.Launcher.isFeederJammed || this.Launcher.isLauncherJammed)
        {
            Controller.Base.setRumble(RumbleType.kBothRumble, 1);
        }
        else
        {
            Controller.Base.setRumble(RumbleType.kBothRumble, 0);
        }

    }

    @Override
    public void teleopInit() {
        
        Intake.setDefaultCommand(Commands.run(() -> {

            if (Controller.getIntakeButton())
            {
                Intake.intake();
            }
            else if (Controller.getOuttakeButton())
            {
                Intake.outtake();
            }
            else
            {
                Intake.stop();
            }

        }, Intake));

        Drive.setDefaultCommand(Commands.run(() -> {

            var controllerSpeeds = this.Controller.getArcadeChassisSpeeds();

            SmartDashboard.putNumber("Drive/X", controllerSpeeds.vxMetersPerSecond);
            SmartDashboard.putNumber("Drive/Y", controllerSpeeds.vyMetersPerSecond);
            SmartDashboard.putNumber("Drive/Omega", controllerSpeeds.omegaRadiansPerSecond);

            Drive.driveRobotRelative(controllerSpeeds);

        }, Drive));

        Launcher.setDefaultCommand(Commands.run(() -> {

            if (this.Controller.getAutoLaunchButton())
            {
                // AUTO ALIGN TO HUB STAGE AREA
                // RUMBLE IF OUTSIDE OF SHOOTING AREA (ONLY ON THE WRONG SIDE, AUTO WILL FACE HUB, AND GET CLOSER OR FURTHER)

                if (!Constants.isOnLaunchingSideOfAllianceHub(Drive.getEstimatedPose().getTranslation()))
                {
                    Controller.rumbleCmd(1, Duration.ofMillis(250));
                    return;
                }

                GameCommands.autoStageAndLaunchFuelInAllianceHub(this).onlyWhile(() -> Controller.getAutoLaunchButton()).schedule();
            }
            else if (this.Controller.getManualLaunchButton() > 0)
            {
                // RIGHT TRIGGER IS MANUAL FIRE MIN - MAX RPM, IF TRIGGER ZERO, ZERO RPM.

                var manualRPM = Constants.Launcher.MIN_RPM.plus(Constants.Launcher.MAX_RPM.minus(Constants.Launcher.MIN_RPM)).times(this.Controller.getManualLaunchButton());

                Launcher.controlMode = LaunchControlMode.RPM;
                Launcher.desiredRPM = manualRPM;
                Launcher.doFeed = true;
            }
            else
            {
                Launcher.stop();
            }

        }, Launcher));
    
    }

    private int testClimberVolts = 5;

    @Override
    public void testInit() 
    {
        Launcher.setDefaultCommand(Commands.run(() -> {

            // TEST LAUNCHER FEEDER
            Launcher.doFeed = Controller.Base.getLeftTriggerAxis() > 0.2;

            var rightThrottle = MathUtil.applyDeadband(Controller.Base.getRightTriggerAxis(), 0.1);

            Launcher.controlMode = LaunchControlMode.RPM;
            Launcher.desiredRPM =   Constants.Launcher.LAUNCHER_FREE_SPINNING_VELOCITY.times(rightThrottle);

            if (Controller.Base.getLeftBumperButton())
            {
                // TEST MAX SPEED TO CALCULATE FREE SPINNING MAX SPEED AND ETC.

                Launcher.controlMode = LaunchControlMode.Voltage;
                Launcher.desiredVoltage = 12;
            }

        }, Launcher));

        // Climber.setDefaultCommand(Commands.run(() -> {

        //     SmartDashboard.putNumber("Climber/TestClimberVolts", testClimberVolts);

        //     if (Controller.Base.getPOV() == 0)
        //     {
        //         // Climber UP
        //         Climber.setVoltage(testClimberVolts);
        //     }
        //     else if (Controller.Base.getPOV() == 90)
        //     {
        //         // Increase test Volts
        //         testClimberVolts += 1;
        //     }
        //     else if (Controller.Base.getPOV() == 270)
        //     {
        //         // Decrease test Volts
        //         testClimberVolts -= 1;
        //     }
        //     else if (Controller.Base.getPOV() == 180)
        //     {
        //         // Climber DOWN
        //         Climber.setVoltage(-testClimberVolts);
        //     }
        //     else
        //     {
        //         Climber.stop();
        //     }

        // }, Climber));
    }

    @Override
    public void testPeriodic() {

        this.Drive.Stop();
        
        if (Controller.Base.getBButtonPressed())
        {
            var launcherTuningCommand = new LauncherTuningCommand(this.Launcher, 
                RPM.of(1000), // Min
                Constants.Launcher.LAUNCHER_FREE_SPINNING_VELOCITY, // Max
                RPM.of(50),  // Interval
                Controller.Base::getYButtonPressed,
                Controller.Base::getAButtonPressed,
                Controller.Base::getXButtonPressed);

            launcherTuningCommand.unless(() -> Controller.Base.getBButtonPressed()).schedule();
        }

    }
}
