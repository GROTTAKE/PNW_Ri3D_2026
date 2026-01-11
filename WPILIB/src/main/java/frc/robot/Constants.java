package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import java.time.Duration;
import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.models.CircularStage;

public class Constants
{
    public static class Vehicle
    {
        public static final Distance RobotRadius = Inches.of(26 / 2); 
        public static final double WeightInKG = 90;
        public static final double MOI = 7.5;
    }

    public static class Drive
    {
        public static final double WHEEL_FREE_SPIN_RPS = 8.1;

        public static final Distance TRACK_WIDTH = Inches.of(23 + 1 / 8);
        public static final Distance WHEEL_DIAMETER = Inches.of(6);
        public static final Distance WHEEL_RADIUS = WHEEL_DIAMETER.div(2);
        public static final Distance WHEEL_CIRCUMFERENCE = Meters.of(2 * Math.PI * (WHEEL_DIAMETER.in(Meters) / 2));
        public static final double GEAR_RATIO = 8.45; 

        public static final int LEFT_LEAD_CAN_ID = 1;
        public static final int LEFT_FOLLOW_CAN_ID = 2;
        public static final int RIGHT_LEAD_CAN_ID = 3;
        public static final int RIGHT_FOLLOW_CAN_ID = 4;

        public static final LinearVelocity MAX_TRANSLATION_VELOCITY = MetersPerSecond.of(1);
        public static final LinearVelocity MAX_TRANSLATION_AUTO_VELOCITY = MetersPerSecond.of(0.5);

        public static final AngularVelocity MAX_ANGULAR_VELOCITY = DegreesPerSecond.of(90);
        
        public static final AngularAcceleration MAX_ANGULAR_ACCEL = DegreesPerSecondPerSecond.of(90);

        public static final LinearAcceleration MAX_TRANSLATION_ACCEL = MetersPerSecondPerSecond.of(0.5);
    }

    public static class Launcher 
    {
        public static final int LAUNCH_CAN_ID = 11;
        public static final int FEEDER_CAN_ID = 10;
        public static final int MAX_ACCEL_RPM = 100;

        public static final AngularVelocity MIN_RPM = RPM.of(2700); 
        public static final AngularVelocity MAX_RPM = RPM.of(3800);

        public static final AngularVelocity LAUNCHER_FREE_SPINNING_VELOCITY = RPM.of(5700);

        // public static final double P = 0.00065;
        // public static final double I = 0;
        // public static final double D = 0.001;

        public static final double P = 0.00060;
        public static final double I = 0;
        public static final double D = 0;

        public static final Duration LAUNCHER_RAMP_FULL_SECONDS = Duration.ofSeconds(2);
        public static final AngularVelocity LAUNCHER_MAX_RAMP_VELOCITY = LAUNCHER_FREE_SPINNING_VELOCITY.div(LAUNCHER_RAMP_FULL_SECONDS.toSeconds());

        public static final Duration JAM_UNDO_DURATION = Duration.ofMillis(2000);

        public static Optional<AngularVelocity> calculateLaunchRPM(Distance distanceFromHUB)
        {
            // https://docs.google.com/spreadsheets/d/1r6IOnGSkYCTGS0Y9jjmeL8ff74aTfAI9nv18nynvWNY/edit?usp=sharing
            
            double x = distanceFromHUB.in(Meters);

            var requiredRPM = RPM.of(3294 + -849 * x + 267 * Math.pow(x, 2) + 53 * Math.pow(x, 3) + -14 * Math.pow(x, 4));
            // var requiredRPM = RPM.of(2223 * Math.pow(Math.E, 0.133 * x));

            if (requiredRPM.lt(MIN_RPM) || requiredRPM.gt(MAX_RPM)) return Optional.empty();

            return Optional.of(requiredRPM);
        }

        public static Distance MIN_SHOOTING_DISTANCE = Meters.of(1.5);
        public static Distance MAX_SHOOTING_DISTANCE = Meters.of(4);

        public static CircularStage BLUE_HUB_STAGE = new CircularStage(Field_BLUE.BLUE_HUB, MIN_SHOOTING_DISTANCE, MAX_SHOOTING_DISTANCE);
        public static CircularStage RED_HUB_STAGE = new CircularStage(Field_BLUE.RED_HUB, MIN_SHOOTING_DISTANCE, MAX_SHOOTING_DISTANCE);
    }

    public static class Intake
    {
        public static final int CAN_ID = 20;
        public static int MAX_ACCEL_RPM = 1000;
    }

    public static class Climber
    {
        public static final int CAN_ID = 30;
        public static final double GEAR_RATIO = 100;
        public static final int PULL_VOLTAGE = 10;

        public static final Distance SPROCKET_DIAMETER = Inches.of(1.757);
        public static final Distance SPROCKET_RADIUS = SPROCKET_DIAMETER.div(2);
        public static final Distance SPROCKET_CIRCUMFERENCE = SPROCKET_RADIUS.times(2).times(Math.PI);
    }

    public static class Limelight
    {
      	public static final String Name = "Sauron";
    }

    public static boolean isOnLaunchingSideOfAllianceHub(Translation2d robotPosition)
    {
        var allianceOpt = DriverStation.getAlliance();
        if (allianceOpt.isEmpty()) return false;
    
        var alliance = allianceOpt.get();
        Translation2d hub = alliance == Alliance.Blue ? Field_BLUE.BLUE_HUB : Field_BLUE.RED_HUB;
        
        boolean isOnCorrectSide = alliance == Alliance.Blue
            ? robotPosition.getX() <= hub.getX() - Field_BLUE.HUB_WIDTH.in(Meters) / 2 - Vehicle.RobotRadius.in(Meters)
            : robotPosition.getX() >= hub.getX() + Field_BLUE.HUB_WIDTH.in(Meters) / 2 + Vehicle.RobotRadius.in(Meters);
        
        return isOnCorrectSide;
    }
    
    public static boolean isInsideShootingArea(Translation2d robotPosition)
    {
        var allianceOpt = DriverStation.getAlliance();
        if (allianceOpt.isEmpty()) return false;
    
        var alliance = allianceOpt.get();
        Translation2d hub = alliance == Alliance.Blue ? Field_BLUE.BLUE_HUB : Field_BLUE.RED_HUB;
    
        double distanceFromHub = hub.getDistance(robotPosition);
    
        return isOnLaunchingSideOfAllianceHub(robotPosition)
            && distanceFromHub >= Launcher.MIN_SHOOTING_DISTANCE.in(Meters)
            && distanceFromHub <= Launcher.MAX_SHOOTING_DISTANCE.in(Meters);
    }
    
    public static class Field_BLUE
    {
        private static final Distance FIELD_WIDTH = Inches.of(651.22);

        private static final Distance HUB_Y = Meters.of(4.076);
        private static final Distance HUB_X_FROM_WALL = Meters.of(4.636);

        public static final Distance HUB_WIDTH = Inches.of(47);

        public static Translation2d BLUE_HUB = new Translation2d(HUB_X_FROM_WALL, HUB_Y);
        public static Translation2d RED_HUB = new Translation2d(FIELD_WIDTH.minus(HUB_X_FROM_WALL), HUB_Y);
    }
}