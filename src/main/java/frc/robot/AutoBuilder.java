package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public final class AutoBuilder {
    public static final Command doNothing(SwerveSubsystem swerve) {
        return Commands.none();
    }

    /**
     * Every auto routine needs this command
     *
     * @param swerve
     * @return
     */
    private static final Command prefix(SwerveSubsystem swerve) {
        return Commands.none();
    }
}
