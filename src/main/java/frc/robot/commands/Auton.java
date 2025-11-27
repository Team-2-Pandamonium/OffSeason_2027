package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants;

public class Auton extends SubsystemBase {
    /**
     * 
     * @param Rot
     * @return linDist
     */
    public static double rotToDist(double wheelRot) {
        double linDist = wheelRot * (Math.PI * 6);
        return linDist/8.46;
    }

    /**
     * finds the difference between current x and y, to the desired x and y, if "x" inputted, gives the difference in x, else gives "y"
     * desX and Y are the desired x and y, please make them in inches
     * @param des the desired value, x or y
     * @param xy return either "x" or "y" difference from the desired value
     * @return
    */
    public static double desToCurrentDifference(double des, String xy) {
        double yAVG = rotToDist((((Robot.drvL1Enc.getPosition()+Robot.drvR1Enc.getPosition())/2) //avg of front side
          +  ((Robot.drvL2Enc.getPosition()+Robot.drvR2Enc.getPosition())/2)) //avg of back side
          /2); //avg of the avg's
        double xAVG = rotToDist((((Robot.drvR1Enc.getPosition()+Robot.drvR2Enc.getPosition())/2) //avg of right side
        +  ((Robot.drvL1Enc.getPosition()+Robot.drvL2Enc.getPosition())/2)) //avg of left side
        /2); //avg of the avg's

        if (xy=="x") {
            return des-xAVG;
        } else {
            return des-yAVG;
        }
    }

    public static void FinalCommandToRunConstantly(double desX, double desY) {
        if (Math.abs(desToCurrentDifference(desX, "x"))>1) {
            Robot.right1.set(Math.copySign(0.5, desToCurrentDifference(desX, "x")));
            Robot.right2.set(Math.copySign(0.5, desToCurrentDifference(desX, "x")));
        } else if (Math.abs(desToCurrentDifference(desY, "y"))>1) {
            Robot.right2.set(0);

            Robot.left1.set(Math.copySign(0.5, desToCurrentDifference(desY, "y")));
            Robot.right1.set(Math.copySign(0.5, desToCurrentDifference(desY, "y")));
        } else {
            Robot.left1.set(0);
            Robot.right1.set(0);
            System.out.print("HAS REACHED THE SPECIFIED COORDINATE");
        }
    }

}

