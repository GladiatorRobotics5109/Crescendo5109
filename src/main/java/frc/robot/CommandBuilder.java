package frc.robot;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.winch.WinchSubsystem;
import frc.robot.util.Conversions;
import frc.robot.util.Util;

public final class CommandBuilder {
    private static CommandBuilder s_instance;

    public static void init(
        SwerveSubsystem swerve,
        ShooterSubsystem shooter,
        WinchSubsystem winch,
        Rollers rollers,
        CommandGenericHID driverController
    ) {
        s_instance = new CommandBuilder(swerve, shooter, winch, rollers, driverController);
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

    public static Command commandDriverControllerNoteEnterSequence() {
        return s_instance.commandDriverControllerNoteEnterSequenceImpl();
    }

    private final SwerveSubsystem m_swerve;
    private final ShooterSubsystem m_shooter;
    private final WinchSubsystem m_winch;
    private final Rollers m_rollers;

    private final CommandGenericHID m_driverController;

    private CommandBuilder(
        SwerveSubsystem swerve,
        ShooterSubsystem shooter,
        WinchSubsystem winch,
        Rollers rollers,
        CommandGenericHID driverController
    ) {
        m_swerve = swerve;
        m_shooter = shooter;
        m_winch = winch;
        m_rollers = rollers;
        m_driverController = driverController;
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
            // Get bot in desired state
            commandStopIntakeImpl(),
            m_shooter.commandManualStart(),
            Commands.print("Starting shooter..."),
            Commands.waitUntil(m_shooter::isAtDesiredRPM).withTimeout(3),
            // Start feeders
            m_rollers.commandStartManualShoot(),
            Commands.print("Starting feeder..."),
            // Wait till note leaves
            Commands.waitUntil(() -> !m_rollers.hasNote()).withTimeout(2),
            commandStopManualShootImpl(),
            Commands.print("DONE")
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

    private Command commandDriverControllerNoteEnterSequenceImpl() {
        Timer timer = new Timer();

        return new FunctionalCommand(
            // init
            () -> {
                timer.reset();

                // Logger.recordOutput("Controllers/RumbleValue", 0);
                Logger.recordOutput("Controllers/ElapsedTime", timer.get());
            },
            // execute
            () -> {
                timer.start();

                double value = MathUtil.clamp(
                    Math.sin(
                        Conversions.rotationsToRadians(
                            (timer.get() / Constants.DriveTeamConstants.kDriverOnNoteEnterRumbleDurationSecs) / 2
                        )
                    ),
                    0.0,
                    1.0
                );

                Logger.recordOutput("Controllers/RumbleValue", value);
                Logger.recordOutput("Controllers/ElapsedTime", timer.get());
                m_driverController.getHID().setRumble(
                    RumbleType.kBothRumble,
                    value
                );
            },
            // end
            interrupted -> {
                m_driverController.getHID().setRumble(
                    RumbleType.kBothRumble,
                    0.0
                );

                Logger.recordOutput("Controllers/RumbleValue", 0.0);
            },
            // isFinished
            () -> {
                return timer.hasElapsed(Constants.DriveTeamConstants.kDriverOnNoteEnterRumbleDurationSecs);
            }
        ).withName("CommandBuilder::commandDriverControllerNoteEnterSequence");
    }

    /**
     * Constructs a command that polls a boolean supplier and enables/disables override mode.
     *
     * @param enabled
     *            {@link BooleanSupplier} supplier to use
     */
    public static Command commandOverrideModeWatcher(BooleanSupplier enabled) {
        return Commands.run(() -> StateMachine.RobotState.setOverrideModeEnabled(enabled.getAsBoolean())).withName(
            "CommandBuilder::commandOverrideModeWatcher"
        );
    }
}
