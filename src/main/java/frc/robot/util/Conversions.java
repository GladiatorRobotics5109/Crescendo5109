package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public final class Conversions {
    private Conversions() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    // Most of these functions just call their respective functions in the Units utility class because of the conflict
    // in naming between edu.wpi.first.math.util.Units and edu.wpi.first.units.Units classes (I HATE this conflict).

    public static double inchesToMeters(double in) {
        return Units.inchesToMeters(in);
    }

    public static double metersToInches(double m) {
        return Units.metersToInches(m);
    }

    public static double degreesToRadians(double deg) {
        return Units.degreesToRadians(deg);
    }

    public static double radiansToDegrees(double rad) {
        return Units.radiansToDegrees(rad);
    }

    public static double radiansToRotations(double rad) {
        return radiansToRotations(rad, 1);
    }

    public static double radiansToRotations(double rad, double gearRatio) {
        return Units.radiansToRotations(rad * gearRatio);
    }

    public static double rotationsToRadians(double rot) {
        return rotationsToRadians(rot, 1);
    }

    public static double rotationsToRadians(double rot, double gearRatio) {
        return Units.rotationsToRadians(rot * gearRatio);
    }

    public static double rotationsPerMinuteToRadiansPerSecond(double rotPerMin) {
        return rotationsPerMinuteToRadiansPerSecond(rotPerMin, 1);
    }

    public static double rotationsPerMinuteToRadiansPerSecond(double rotPerMin, double gearRatio) {
        return Units.rotationsPerMinuteToRadiansPerSecond(rotPerMin * gearRatio);
    }

    public static double radiansPerSecondToRotationsPerMinute(double radPerSec) {
        return radiansPerSecondToRotationsPerMinute(radPerSec, 1);
    }

    public static double radiansPerSecondToRotationsPerMinute(double radPerSec, double gearRatio) {
        return Units.radiansPerSecondToRotationsPerMinute(radPerSec * gearRatio);
    }

    public static double driveMotorRotationsToDriveWheelRadians(double rot) {
        return rotationsToRadians(rot, 1 / SwerveConstants.SwerveModuleConstants.kDriveGearRatio);
    }

    public static double radiansToMeters(double rad, double radiusM) {
        return radiansToMeters(rad, radiusM, 1);
    }

    public static double radiansToMeters(double rad, double radiusM, double gearRatio) {
        return (rad * gearRatio) * radiusM;
    }

    public static double metersToRadians(double m, double radiusM, double gearRatio) {
        return (m * gearRatio) / radiusM;
    }

    public static double wheelMetersToDriveMotorRadians(double m) {
        return metersToRadians(
            m,
            SwerveConstants.SwerveModuleConstants.kWheelRadiusMeters,
            SwerveConstants.SwerveModuleConstants.kDriveGearRatio
        );
    }

    public static double turnMotorRotationsToWheelRadians(double rot) {
        return rotationsToRadians(rot, 1 / SwerveConstants.SwerveModuleConstants.kTurnGearRatio);
    }

    public static Rotation2d turnMotorRotationsToWheelRotation2d(double rot) {
        return Rotation2d.fromRadians(turnMotorRotationsToWheelRadians(rot));
    }

    public static double wheelRadiansToWheelMeters(double rad) {
        return radiansToMeters(rad, SwerveConstants.SwerveModuleConstants.kWheelRadiusMeters);
    }

    public static double wheelMetersToWheelRadians(double m) {
        return metersToRadians(m, SwerveConstants.SwerveModuleConstants.kWheelRadiusMeters, 1);
    }

    public static double shooterRotationsPerMinuteToShooterRadiansPerSecond(double rpm) {
        return rotationsPerMinuteToRadiansPerSecond(rpm);
    }

    public static double shooterRadiansPerSecondToShooterRotationsPerMinute(double radPerSec) {
        return radiansPerSecondToRotationsPerMinute(radPerSec);
    }

    public static double intakeMotorRotationsToIntakeRadians(double rot) {
        return rotationsToRadians(rot, 1 / Constants.RollersConstants.IntakeConstants.kIntakeGearRatio);
    }

    public static double intakeMotorRotationsPerMinuteToIntakeRadiansPerSecond(double rotPerMin) {
        return rotationsPerMinuteToRadiansPerSecond(
            rotPerMin,
            1 / Constants.RollersConstants.IntakeConstants.kIntakeGearRatio
        );
    }
}
