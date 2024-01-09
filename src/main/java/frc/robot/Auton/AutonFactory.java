package frc.robot.Auton;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutonFactory {
    public static Command getDefaultAutoCommand(SwerveSubsystem swerve) {
        // TODO: test this
        // generate list of test poses to go to
        List<Translation2d> poses = PathPlannerPath.bezierFromPoses(
            new Pose2d(),
            new Pose2d(
                new Translation2d(0, 3),
                Rotation2d.fromRadians(Math.PI / 2)
            ),
            new Pose2d()
        );

        // create bath based off poses
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

        // return command object that drive with a trajectory
        return Commands.sequence(
            swerve.getDriveWithPathCommand(path)
        );
    }
}
