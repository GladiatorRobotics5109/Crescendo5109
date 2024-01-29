package frc.robot.auton;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutonFactory {
    public static Command getTestAuton(SwerveSubsystem swerve) {
        PathPlannerPath path = PathPlannerPath.fromPathFile("TestPath");
    PathPlannerPath path2 = PathPlannerPath.fromPathFile("TestPath2");

    if (path == null || path2 == null) {
      System.out.println("NO PATH!!!!!!!!");
    }

    return Commands.sequence(
      swerve.getFollowPathCommand(path),
      Commands.waitSeconds(2),
      swerve.getFollowPathCommand(path2)
    );
    }
}
