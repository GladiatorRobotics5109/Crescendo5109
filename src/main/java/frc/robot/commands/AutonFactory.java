package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public final class AutonFactory {
    public static Command getTaxiCommand(SwerveSubsystem swerve) {
        return Commands.sequence(
            Commands.runOnce(() -> swerve.drive(0, 1, 0, true), swerve),
            Commands.waitSeconds(1.25),
            Commands.runOnce(() -> swerve.drive(0, 0, 0, true), swerve)
        );
    }

    public static Command getShootAndTaxiCommand(SwerveSubsystem swerve, ShooterSubsystem shooter) {
        return Commands.sequence(
            getTaxiCommand(swerve),
            shooter.getStartShooterCommand(),
            shooter.getAimAmpCommand(),
            Commands.waitSeconds(3),
            shooter.getStartFeederCommand(),
            Commands.waitSeconds(2),
            shooter.getStopFeederCommand(),
            shooter.getStopShooterCommand()
        );
    }
}
