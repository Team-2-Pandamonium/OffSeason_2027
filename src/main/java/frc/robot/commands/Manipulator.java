package frc.robot.commands;

public class Manipulator {

    /**
     * 
     * @param drvRot
     * @return <b>linVel<b> (inches)
     */
    public static double drvRotLinVel(double drvMotRot){
        double wheelRot=drvMotRot/8.46;
        return wheelRot*(Math.PI*6);
    }

    /**
     * 
     * @param manMotRot
     * @return <b>linVel<b> (inches)
     */
    public static double manRotLinVel(double manMotRot){
        double wheelRot=manMotRot/4;
        return wheelRot*(Math.PI*4);
    }
    
}