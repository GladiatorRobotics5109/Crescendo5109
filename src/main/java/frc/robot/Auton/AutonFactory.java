package frc.robot.Auton;

import java.util.List;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutonFactory {
    // TODO: test this
    public static Command getDefaultAutoCommand(SwerveSubsystem swerve) {
        List<Translation2d> poses = PathPlannerPath.bezierFromPoses(
            new Pose2d(),
            new Pose2d(
                new Translation2d(0, 3),
                Rotation2d.fromRadians(Math.PI / 2)
            ),
            new Pose2d()
        );

        PathPlannerPath path = new PathPlannerPath(
            poses, 
            new PathConstraints(
                Constants.SwerveConstants.kDefaultSpeed, 
                Constants.SwerveConstants.kMaxAngularAcceleration, 
                Constants.SwerveConstants.kMaxAngularSpeed,
                Constants.SwerveConstants.kMaxAngularAcceleration
            ),
            new GoalEndState(0, Rotation2d.fromRadians(0))
        );

        return Commands.sequence(
            swerve.getDriveWithTrajectoryCommand(
                () -> Timer.getFPGATimestamp(), 
                path.getTrajectory(
                    new ChassisSpeeds(), 
                    swerve.getHeading()
                )
            )
        );
    }
}
