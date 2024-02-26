package frc.robot;

public class Conversions {

    public static double rotToMeters(double rot) {
        return rot * (Constants.ModuleConstants.kWheelRadius * 2 * Math.PI) / Constants.ModuleConstants.kSwerveDriveGearRatio;
    }

    public static double metersToRot(double meters) {
        return meters  * (Constants.ModuleConstants.kSwerveDriveGearRatio / (2 * Math.PI * Constants.ModuleConstants.kWheelRadius));
    }
    
}
