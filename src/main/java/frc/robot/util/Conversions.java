package frc.robot.util;

import com.github.gladiatorrobotics5109.gladiatorroboticslib.math.ConversionsBase;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public final class Conversions extends ConversionsBase {
    private Conversions() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    public static double driveMotorRotationsToDriveWheelRadians(double rot) {
        return Conversions.driveMotorRotationsToDriveWheelRadians(
            rot,
            Constants.SwerveConstants.SwerveModuleConstants.kDriveGearRatioEnum
        );
    }

    public static double wheelMetersToDriveMotorRadians(double m) {
        return Conversions.wheelMetersToDriveMotorRadians(
            m,
            Constants.SwerveConstants.SwerveModuleConstants.kWheelRadiusMeters,
            Constants.SwerveConstants.SwerveModuleConstants.kDriveGearRatioEnum
        );
    }

    public static Rotation2d turnMotorRotationsToWheelRotation2d(double rot) {
        return Rotation2d.fromRadians(turnMotorRotationsToWheelRadians(rot));
    }

    public static double wheelRadiansToWheelMeters(double rad) {
        return Conversions.wheelRadiansToWheelMeters(rad, SwerveConstants.SwerveModuleConstants.kWheelRadiusMeters);
    }

    public static double wheelMetersToWheelRadians(double m) {
        return Conversions.metersToRadians(m, SwerveConstants.SwerveModuleConstants.kWheelRadiusMeters, 1);
    }

    public static double shooterRotationsPerMinuteToShooterRadiansPerSecond(double rpm) {
        return Conversions.rotationsPerMinuteToRadiansPerSecond(rpm);
    }

    public static double shooterRadiansPerSecondToShooterRotationsPerMinute(double radPerSec) {
        return Conversions.radiansPerSecondToRotationsPerMinute(radPerSec);
    }

    public static double intakeMotorRotationsToIntakeRadians(double rot) {
        return Conversions.rotationsToRadians(rot, 1 / Constants.RollersConstants.IntakeConstants.kIntakeGearRatio);
    }

    public static double intakeMotorRotationsPerMinuteToIntakeRadiansPerSecond(double rotPerMin) {
        return Conversions.rotationsPerMinuteToRadiansPerSecond(
            rotPerMin,
            1 / Constants.RollersConstants.IntakeConstants.kIntakeGearRatio
        );
    }

    public static Rotation2d winchMotorRadiansToWinchAngle(double rad) {
        return Rotation2d.fromDegrees(
            57.8763 + (-1.07687 * Conversions.radiansToRotations(rad))
        );
    }

    public static double winchAngleToWinchMotorRadians(Rotation2d angle) {
        // angleDeg = 57.8763 + (-1.07687 * Conversions.radiansToRotations(rot))
        // angleDeg - 57.8763 = -1.07687 * Conversions.radiansToRotations(rot)
        // (angleDeg - 57.8763) / -1.07687 = Conversions.radiansToRotations(rot)
        return Conversions.rotationsToRadians((angle.getDegrees() - 57.8763) / -1.07687);
    }
}
