package frc.robot.util;

public class Conversions {

    public static double rotToMeters(double rot) {
        return rot * (Constants.ModuleConstants.kWheelRadius * 2 * Math.PI) / Constants.ModuleConstants.kSwerveDriveGearRatio;
    }

    public static double wheelRotToMeters(double wheelRot) {
        return wheelRot * (Constants.ModuleConstants.kWheelRadius * 2 * Math.PI); // rotations x circumference = meters
    }

    public static double metersToRot(double meters) {
        return meters  * (Constants.ModuleConstants.kSwerveDriveGearRatio / (2 * Math.PI * Constants.ModuleConstants.kWheelRadius));
    }

    public static double metersToWheelRot(double meters) {
        return meters / (Constants.ModuleConstants.kWheelRadius * 2 * Math.PI); // meters / circumference = rotations
    }
    
}
