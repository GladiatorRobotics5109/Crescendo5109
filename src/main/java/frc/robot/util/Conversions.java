package frc.robot.util;

public class Conversions {

    public static double wheelToMeters(double wheel) {
        return wheel * (Constants.ModuleConstants.kWheelRadius * 2 * Math.PI);
    }
    
}