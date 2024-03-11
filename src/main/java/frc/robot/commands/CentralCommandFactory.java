package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public final class CentralCommandFactory {
    private static CentralCommandFactory s_instance;

    private final IntakeSubsystem m_intakeSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
    private final SwerveSubsystem m_swerveSubsystem;
    private final ClimbSubsystem m_climbSubsystem;

    private CentralCommandFactory(
        IntakeSubsystem intakeSubsystem,
        ShooterSubsystem shooterSubsystem,
        SwerveSubsystem swerveSubsystem,
        ClimbSubsystem climbSubsystem
    ) {
        m_intakeSubsystem = intakeSubsystem;
        m_shooterSubsystem = shooterSubsystem;
        m_swerveSubsystem = swerveSubsystem;
        m_climbSubsystem = climbSubsystem;
    }

    public static void init(IntakeSubsystem intake, ShooterSubsystem shooter, SwerveSubsystem swerve, ClimbSubsystem climb) {
        s_instance = new CentralCommandFactory(intake, shooter, swerve, climb);
    }

    public static Command getStartIntakeAndFeederCommand() {
        return  s_instance.getStartIntakeAndFeederCommandImpl();
    }

    public static Command getToggleIntakeAndFeederCommand() {
        return s_instance.getToggleIntakeAndFeederCommandImpl();
    }

    public static Command getFeederSensorTrueCommand() {
        return s_instance.getFeederSensorTrueCommandImpl();
    }

    public static Command getStartAutoAimCommand() {
        return s_instance.getStartAutoAimCommandImpl();
    }

    public static Command getStopAutoAimCommand() {
        return s_instance.getStopAutoAimCommandImpl();
    }

    public static Command getToggleAutoAimCommand() {
        return s_instance.getToggleAutoAimCommandImpl();
    }

    private Command getStartIntakeAndFeederCommandImpl() {
        return Commands.parallel(
            m_intakeSubsystem.getStartIntakeCommand(),
            m_shooterSubsystem.getStartFeederCommand()
        );
    }

    private Command getToggleIntakeAndFeederCommandImpl() {
        return Commands.parallel(
            m_intakeSubsystem.getToggleIntakeCommand(),
            m_shooterSubsystem.getToggleFeederCommand()
        );
    }
 
    private Command getFeederSensorTrueCommandImpl() {
        return Commands.parallel(
           m_intakeSubsystem.getStopIntakeCommand(),
            m_shooterSubsystem.getStopFeederCommand(),
            m_shooterSubsystem.getStartShooterCommand()
        );
    }

    private Command getStartAutoAimCommandImpl() {
        return Commands.parallel(
            m_shooterSubsystem.getStartAutoAimCommand(),
            m_swerveSubsystem.getStartAutoAimCommand()
        );
    }

    private Command getStopAutoAimCommandImpl() {
        return Commands.parallel(
            m_shooterSubsystem.getStopAutoAimCommand(),
            m_swerveSubsystem.getStopAutoAimCommand()
        );
    }

    private Command getToggleAutoAimCommandImpl() {
        return Commands.parallel(
            m_shooterSubsystem.getToggleAutoAimCommand(),
            m_swerveSubsystem.getToggleAutoAimCommand()
        );
    }
}
