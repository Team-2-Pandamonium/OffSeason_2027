package frc.robot.constants;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class RobotConstants {

    // public static final double leftDrift = 1; // at least one drift should be 1,
    // both if drivebase if fine.
    // public static final double rightDrift = 1;
    public static final double autonSpeed = .5;
    public static final double deadzone = .05;

    // driving
    public static double robotSpeed = 0.5;
    public static boolean cruiseControl = false; // fasle=off true=on

    // elevator (all values in inches)
    public static final double elevator_speed = 0.5;
    public static final double intake_speed = 0.5;
    public static double elevatorHeight;
    public static final double Level1 = 24;
    public static final double Level2 = 24 + 17;
    public static final double Level3 = 24 + 17 * 2;
    public static final double humanPlayer = 35;
    public static double encoderPos;
    public static double encoderRat;
    public static boolean carrigeTop;
    public static boolean stg2Top;
    public static boolean carrigeBot;
    // manipulaor

    // inputs
    public static double rightTrigger;
    public static double leftStick;
    public static double rightStick;
    public static Boolean leftBumper;
    public static Boolean rightBumper;
    public static double leftTrigger;
    public static boolean bButton;
    public static boolean xButton;
    public static boolean aButton;

}
