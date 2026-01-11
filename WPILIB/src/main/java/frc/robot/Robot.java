// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This is a demo program showing the use of the DifferentialDrive class,
 * specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {

    private XboxController DriveController;
    private DriveSubsystem Drive;

    /** Called once at the beginning of the robot program. */
    public Robot() {

        this.DriveController = new XboxController(0);
        this.Drive = new DriveSubsystem();
  
    }

    @Override
    public void robotInit() {

        Drive.setDefaultCommand(Commands.run(() -> {

            var controllerSpeeds = new ChassisSpeeds(
                MathUtil.applyDeadband(-DriveController.getLeftX(), 0.1) * Constants.Drive.MAX_TRANSLATION_VELOCITY.in(MetersPerSecond),
                0,
                MathUtil.applyDeadband(DriveController.getRightX(), 0.1) * Constants.Drive.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond)
            );

            Drive.driveRobotRelative(controllerSpeeds);

        }, Drive));

    }

    @Override
    public void disabledInit() 
    {
        this.Drive.stop();    
    }

    @Override
    public void robotPeriodic() {



    }

    @Override
    public void teleopPeriodic() {
    }
}
