package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.stateMachine.StateMachine;

public class Util {
    /**
     * Guarantees that Alliance will be a valid value by returning
     * Constants.kDefaultAlliance if DriverStation.getAlliance() is empty
     *
     * @return an {@link Alliance} object instead of the optional returned by
     *         DriverStation.getAlliance()
     */
    public static Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(Constants.kDefaultAlliance);
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
}
