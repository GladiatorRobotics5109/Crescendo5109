package frc.robot.util;

/**
 * Utility class for converting units
 */
public class Conversions {
    public static double wheelRotToWheelM(double rot) {
        return rot * (Constants.ModuleConstants.kWheelRadius * 2 * Math.PI);
    }
    
    public static double driveMotorRotToWheelM(double rot) {
        return rot * (Constants.ModuleConstants.kWheelRadius * 2 * Math.PI) / Constants.ModuleConstants.kSwerveDriveGearRatio;
    }

    public static double swerveModuleMToDriveMotorRot(double meters) {
        return meters  * (Constants.ModuleConstants.kSwerveDriveGearRatio / (2 * Math.PI * Constants.ModuleConstants.kWheelRadius));
    }
}
