package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants
{
    public static class Drive
    {
        public static final Distance TRACK_WIDTH = Inches.of(20);
        public static final Distance WHEEL_DIAMETER = Meters.of(0.1524);
        public static final double GEAR_RATIO = 10.71; 

        public static final Distance DISTANCE_PER_MOTOR_REV = Meters.of((Math.PI * WHEEL_DIAMETER.in(Meters)) / GEAR_RATIO);

        public static final int PORT_A_CAN_ID = 1;
        public static final int PORT_B_CAN_ID = 2;
        public static final int STARBOARD_A_CAN_ID = 3;
        public static final int STARBOARD_B_CAN_ID = 4;

        public static final LinearVelocity MAX_TRANSLATION_VELOCITY = MetersPerSecond.of(2);
        public static final LinearVelocity MAX_TRANSLATION_AUTO_VELOCITY = MetersPerSecond.of(1);

        public static final AngularVelocity MAX_ANGULAR_VELOCITY = DegreesPerSecond.of(90);
        
        public static final AngularAcceleration MAX_ANGULAR_ACCEL = DegreesPerSecondPerSecond.of(45);
        public static final LinearAcceleration MAX_TRANSLATION_ACCEL = MetersPerSecondPerSecond.of(0.5);

        public static final AngularVelocity MAX_TRANSLATION_ACCEL_RPM = RPM.of(MAX_TRANSLATION_ACCEL.in(MetersPerSecondPerSecond) / DISTANCE_PER_MOTOR_REV.in(Meters) * 60.0);

    }

    public static class Launcher 
    {
        public static final int CAN_ID = 10;
    }

    public static class Limelight
    {
      	public static final String Name = "Bob";
    }
}