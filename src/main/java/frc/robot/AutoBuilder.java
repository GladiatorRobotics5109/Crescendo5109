package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public final class AutoBuilder {
    public static Command doNothing() {
        return Commands.none();
    }

    public static Command test(SwerveSubsystem swerve) {
        ChoreoTrajectory traj = Choreo.getTrajectory("test");

        return Commands.sequence(
            prefix(swerve, traj.getInitialPose()),
            swerve.followTrajectoryCommand(traj)
        );
    }

    /**
     * Every auto routine needs this command
     */
    private static Command prefix(SwerveSubsystem swerve, Pose2d startingPose) {
        return swerve.setPoseCommand(startingPose);
    }
}
