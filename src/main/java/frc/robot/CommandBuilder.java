package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.winch.WinchSubsystem;
import frc.robot.util.Util;

public final class CommandBuilder {
    private static CommandBuilder s_instance;

    public static void init(
        SwerveSubsystem swerve,
        VisionSubsystem vision,
        ShooterSubsystem shooter,
        WinchSubsystem winch,
        Rollers rollers
    ) {
        s_instance = new CommandBuilder(swerve, vision, shooter, winch, rollers);
    }

    public static Command commandStartIntake() {
        return s_instance.commandStartIntakeImpl();
    }

    public static Command commandStopIntake() {
        return s_instance.commandStopIntakeImpl();
    }

    public static Command commandToggleIntake() {
        return s_instance.commandToggleIntakeImpl();
    }

    public static Command commandEnableAutoAim() {
        return s_instance.commandEnableAutoAimImpl();
    }

    public static Command commandDisableAutoAim() {
        return s_instance.commandDisableAutoAimImpl();
    }

    public static Command commandToggleAutoAim() {
        return s_instance.commandToggleAutoAimImpl();
    }

    public static Command commandStartManualShoot() {
        return s_instance.commandStartManualShootImpl();
    }

    public static Command commandStopManualShoot() {
        return s_instance.commandStopManualShootImpl();
    }

    public static Command commandToggleManualShoot() {
        return s_instance.commandToggleManualShootImpl();
    }

    public static Command commandStartAssistedShoot() {
        return s_instance.commandStartAssistedShootImpl();
    }

    public static Command commandStopAssistedShoot() {
        return s_instance.commandStopAssistedShootImpl();
    }

    public static Command commandToggleAssistedShoot() {
        return s_instance.commandToggleAssistedShootImpl();
    }

    private final SwerveSubsystem m_swerve;
    private final VisionSubsystem m_vision;
    private final ShooterSubsystem m_shooter;
    private final WinchSubsystem m_winch;
    private final Rollers m_rollers;

    private CommandBuilder(
        SwerveSubsystem swerve,
        VisionSubsystem vision,
        ShooterSubsystem shooter,
        WinchSubsystem winch,
        Rollers rollers
    ) {
        m_swerve = swerve;
        m_vision = vision;
        m_shooter = shooter;
        m_winch = winch;
        m_rollers = rollers;
    }

    private Command commandStartIntakeImpl() {
        return Commands.sequence(
            commandDisableAutoAimImpl(),
            Commands.parallel(
                m_winch.commandSetTargetAngleEnabled(() -> true, () -> Constants.WinchConstants.kIntakeAngle),
                m_swerve.commandSetTargetHeadingEnabled(false),
                m_shooter.commandSetAutoSpinUpEnabled(false).andThen(m_shooter.commandStop()).asProxy(),
                m_rollers.commandStartIntakeProcedure().asProxy()
            ),
            commandStopIntakeImpl()
        ).withName("CommandBuilder::commandStartIntake").handleInterrupt(
            () -> DriverStation.reportWarning("CommandBuilder::commandStartIntake has been interrupted!", false)
        ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    private Command commandStopIntakeImpl() {
        return Commands.sequence(
            m_winch.commandSetTargetAngleEnabled(() -> false),
            m_rollers.commandStopIntake(),
            m_shooter.commandSetAutoSpinUpEnabled(true)
        ).withName("CommandBuilder::commandStopIntake");
    }

    private Command commandToggleIntakeImpl() {
        return Commands.either(
            commandStopIntakeImpl(),
            commandStartIntakeImpl(),
            StateMachine.RollersState::isIntaking
        ).withName("CommandBuilder::toggleIntake");
    }

    private Command commandEnableAutoAimImpl() {
        return Commands.sequence(
            commandStopIntakeImpl(),
            Commands.parallel(
                m_swerve.commandSetTargetHeadingEnabled(true, Util::targetHeadingTest),
                m_shooter.commandSetAutoSpinUpEnabled(true),
                // TODO: make target angle method
                m_winch.commandSetTargetAngleEnabled(() -> true, Util::getWinchAutoAimAngle)
            )
        ).withName("CommandBuilder::commandEnableAutoAim");
    }

    private Command commandDisableAutoAimImpl() {
        return Commands.parallel(
            m_swerve.commandSetTargetHeadingEnabled(false),
            m_winch.commandSetTargetAngleEnabled(() -> false)
        ).withName("CommandBuilder::commandDisableAutoAim");
    }

    private Command commandToggleAutoAimImpl() {
        return Commands.either(
            commandDisableAutoAimImpl(),
            commandEnableAutoAimImpl(),
            StateMachine.SwerveState::isTargetingHeading
        ).withName("CommandBuilder::commandToggleAutoAim");
    }

    private Command commandStartManualShootImpl() {
        return Commands.sequence(
            commandStopIntakeImpl(),
            m_rollers.commandStartManualShoot(),
            m_shooter.commandManualStart()
        ).withName("CommandBuilder::commandStartManualShoot");
    }

    private Command commandStopManualShootImpl() {
        return Commands.parallel(
            m_shooter.commandManualStop(),
            m_rollers.commandStopManualShoot()
        ).withName("CommandBuilder::commandStopManualShoot");
    }

    private Command commandToggleManualShootImpl() {
        return Commands.either(
            commandStopManualShootImpl(),
            commandStartManualShootImpl(),
            StateMachine.ShooterState::hasDesiredRPM
        ).withName("CommandBuilder::ToggleManualShoot");
    }

    private Command commandStartAssistedShootImpl() {
        return Commands.sequence(
            commandSetIsAssistedShooting(true),
            commandEnableAutoAimImpl(),
            m_shooter.commandStart(),
            Commands.waitUntil(
                () -> m_shooter.isAtDesiredRPM() && m_swerve.isAtTargetHeading() && m_winch.isAtTargetAngle()
            ).withTimeout(5),
            m_rollers.commandStartShootProcedure(),
            m_shooter.commandStop(),
            commandDisableAutoAim(),
            commandSetIsAssistedShooting(false)
        ).withName("CommandBuilder::commandStartAssistedShoot");
    }

    private Command commandStopAssistedShootImpl() {
        return Commands.sequence(
            m_rollers.commandStopManualShoot(),
            m_shooter.commandStop(),
            commandDisableAutoAimImpl(),
            commandSetIsAssistedShooting(false)
        ).withName("CommandBuilder::commandStopAssistedShoot");
    }

    private Command commandToggleAssistedShootImpl() {
        return Commands.either(
            commandStopAssistedShootImpl(),
            commandStartAssistedShootImpl(),
            StateMachine.RobotState::isAssistedShooting
        ).withName("CommandBuilder::commandToggleAssistedShoot");
    }

    private Command commandSetIsAssistedShooting(boolean isAssistedShooting) {
        return Commands.runOnce(() -> StateMachine.RobotState.setIsAssistedShooting(isAssistedShooting));
    }
}
