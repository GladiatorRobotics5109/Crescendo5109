package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.SwerveConstants;

public final class Conversions {
    private Conversions() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    public static double RadToRot(double rad) {
        return RadToRot(rad, 1);
    }

    public static double RadToRot(double rad, double gearRatio) {
        return Units.radiansToDegrees(rad * gearRatio);
    }

    public static double RotToRad(double rot) {
        return RotToRad(rot, 1);
    }

    public static double RotToRad(double rot, double gearRatio) {
        return Units.rotationsToRadians(rot * gearRatio);
    }

    public static double RotPerMinToRadPerSec(double rotPerMin) {
        return RotPerMinToRadPerSec(rotPerMin, 1);
    }

    public static double RotPerMinToRadPerSec(double rotPerMin, double gearRatio) {
        return Units.rotationsPerMinuteToRadiansPerSecond(rotPerMin * gearRatio);
    }

    public static double DriveMotorRotToDriveWheelRad(double rot) {
        return RotToRad(rot, 1 / SwerveConstants.SwerveModuleConstants.kDriveGearRatio);
    }

    public static double RadToM(double rad, double radiusM) {
        return RadToM(rad, radiusM, 1);
    }

    public static double RadToM(double rad, double radiusM, double gearRatio) {
        return (rad * gearRatio) * radiusM;
    }

    public static double MToRad(double m, double radiusM, double gearRatio) {
        return (m * gearRatio) / radiusM;
    }

    public static double WheelMToDriveMotorRad(double m) {
        return MToRad(
            m,
            SwerveConstants.SwerveModuleConstants.kWheelRadiusMeters,
            SwerveConstants.SwerveModuleConstants.kDriveGearRatio
        );
    }

    public static double TurnMotorRotToWheelRad(double rot) {
        return RotToRad(rot, 1 / SwerveConstants.SwerveModuleConstants.kTurnGearRatio);
    }

    public static Rotation2d TurnMotorRotToWheelRotation2d(double rot) {
        return Rotation2d.fromRadians(TurnMotorRotToWheelRad(rot));
    }

    public static double WheelRadToWheelM(double rad) {
        return RadToM(rad, SwerveConstants.SwerveModuleConstants.kWheelRadiusMeters);
    }

    public static double WheelMToWheelRad(double m) {
        return MToRad(m, SwerveConstants.SwerveModuleConstants.kWheelRadiusMeters, 1);
    }
}
