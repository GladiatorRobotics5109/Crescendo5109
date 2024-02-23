package frc.robot;

public class Conversions {

    public static double wheelToMeters(double wheel) {
        return wheel * (Constants.ModuleConstants.kWheelRadius * 2 * Math.PI);
    }
    
}
