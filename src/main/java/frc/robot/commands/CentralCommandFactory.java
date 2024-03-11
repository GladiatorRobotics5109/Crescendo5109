package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public final class CentralCommandFactory {

    private final IntakeSubsystem m_intakeSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
    private final SwerveSubsystem m_swerveSubsystem;

    public CentralCommandFactory(
        IntakeSubsystem intakeSubsystem,
        ShooterSubsystem shooterSubsystem,
        SwerveSubsystem swerveSubsystem
    ) {
        m_intakeSubsystem = intakeSubsystem;
        m_shooterSubsystem = shooterSubsystem;
        m_swerveSubsystem = swerveSubsystem;
    }

    public Command getStartIntakeAndFeederCommand() {
        return Commands.sequence(
            m_intakeSubsystem.getStartIntakeCommand(),
            m_shooterSubsystem.getStartFeederCommand()
        );
    }

    public Command getToggleIntakeAndFeederCommand() {
        return Commands.sequence(
            m_intakeSubsystem.getToggleIntakeCommand(),
            m_shooterSubsystem.getToggleFeederCommand(),
            m_shooterSubsystem.getSetAngleCommand(38)
        );
    }

    public Command getStartAutoAimCommand() {
        return Commands.sequence(
            m_swerveSubsystem.getStartAutoAimCommand(),
            m_shooterSubsystem.getStartAutoAimCommand(),
            m_shooterSubsystem.getStartShooterCommand()
        );
    }

    public Command getStopAutoAimCommand() {
        return Commands.sequence(
            m_swerveSubsystem.getStopAutoAimCommand(),
            m_shooterSubsystem.getStopAutoAimCommand(),
            m_shooterSubsystem.getStopShooterCommand()
        );
    }
 
    public Command getFeederSensorTrueCommand() {
        return Commands.sequence(
            m_intakeSubsystem.getStopIntakeCommand(),
            m_shooterSubsystem.getStopFeederCommand(),
            m_shooterSubsystem.getStartShooterCommand()
        );
    }

    public Command getReverseAllCommand() {
        return Commands.parallel(
            m_intakeSubsystem.getToggleReverseIntakeCommand(),
            m_shooterSubsystem.getToggleReverseBothCommand()
        );
    }
}
