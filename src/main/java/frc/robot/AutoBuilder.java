package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Util;

public final class AutoBuilder {
    /**
     *
     * @param swerve
     *            reference to swerve subsystem
     * @return Command object that does nothing except sets bot pose to be somewhere near current alliance
     */
    public static Command doNothing(SwerveSubsystem swerve) {
        Alliance alliance = Util.getAlliance();
        // alliance == Alliance.Red, then starting pose is an arbitrary value at (15.0,
        // 4.0, 0deg) which is somewhere near the red alliance, facing the red alliance
        return prefix(
            swerve,
            alliance == Alliance.Red ? new Pose2d(15.0, 4.0, Rotation2d.fromDegrees(0))
                : new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180))
        );
    }

    public static Command test(SwerveSubsystem swerve) {
        PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("test");

        return Commands.sequence(
            prefix(swerve, path.getStartingDifferentialPose()),
            swerve.commandFollowPathPlannerPath(path),
            suffix(swerve)
        );
    }

    /**
     * Every auto routine probably needs to start with this command
     */
    private static Command prefix(SwerveSubsystem swerve, Pose2d startingPose) {
        return swerve.commandSetPose(startingPose);
    }

    /**
     * Every auto routine probably needs to end with this command
     */
    private static Command suffix(SwerveSubsystem swerve) {
        return Commands.sequence(
            swerve.commandSetMaxSpeed(() -> Constants.SwerveConstants.kDefaultSpeed),
            swerve.commandSetTargetHeadingEnabled(false)
        );
    }
}
