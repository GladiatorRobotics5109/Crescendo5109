package frc.robot.util;

import org.gladiatorrobotics.gladiatorroboticslib.UtilBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.stateMachine.StateMachine;

public class Util extends UtilBase {
    public static Alliance getAlliance() {
        return UtilBase.getAlliance(Constants.kDefaultAlliance);
    }

    public static Pose2d getTargetSpeakerPose() {
        switch (Util.getAlliance()) {
            case Red:
                return new Pose2d(
                    Conversions.inchesToMeters(652.73),
                    Conversions.inchesToMeters(218.42),
                    Rotation2d.fromRadians(Math.PI)
                );
            default:
                return new Pose2d(
                    Conversions.inchesToMeters(-1.5),
                    Conversions.inchesToMeters(218.42),
                    Rotation2d.fromRadians(0)
                );
        }
    }

    /**
     * Used for testing heading targeting
     */
    public static Rotation2d targetHeadingTest() {
        // target Speaker code for testing heading control
        Pose2d robotPose = StateMachine.SwerveState.getPose();
        Alliance alliance = Util.getAlliance();
        if (alliance == Alliance.Red) {
            Pose2d redPose = new Pose2d(
                Conversions.inchesToMeters(652.73),
                Conversions.inchesToMeters(218.42),
                Rotation2d.fromRadians(Math.PI)
            );

            Transform2d delta = new Transform2d(
                redPose.getX() - robotPose.getX(),
                redPose.getY() - robotPose.getY(),
                Rotation2d.fromRadians(0)
            );

            return Rotation2d.fromRadians(Math.atan(delta.getY() / delta.getX()));
        }
        else {
            double angleOffset = Math.PI;

            // blue speaker tag pose
            Pose2d bluePose = new Pose2d(
                Conversions.inchesToMeters(-1.5),
                Conversions.inchesToMeters(218.42),
                Rotation2d.fromRadians(0)
            );

            Transform2d delta = new Transform2d(
                robotPose.getX() - bluePose.getX(),
                robotPose.getY() - bluePose.getY(),
                Rotation2d.fromRadians(0)
            );

            return Rotation2d.fromRadians(Math.atan(delta.getY() / delta.getX()) + angleOffset);
        }
    }

    public static Rotation2d getWinchAutoAimAngle() {
        Translation2d targetPose = Util.getTargetSpeakerPose().getTranslation();

        double dist = StateMachine.SwerveState.getPose().getTranslation().getDistance(targetPose);

        double height = Conversions.feetToMeters(6.6) + Conversions.inchesToMeters(5);
        double angle = Math.atan(height / dist);

        // double result = Units.radiansToDegrees(angle) + (2.3 * dist);
        // double result = Units.radiansToDegrees(angle) + (0.6 * dist * dist);
        double result = Conversions.radiansToDegrees(angle) + (0.5 * dist * dist);
        // System.out.println("Auto Aim Request: " + result + "Gravity Compensation: " + (0.5 * dist * dist));

        return Rotation2d.fromDegrees(result);
    }
}
