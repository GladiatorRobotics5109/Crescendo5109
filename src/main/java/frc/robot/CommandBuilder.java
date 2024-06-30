package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.winch.WinchSubsystem;

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

    public static Command startIntake() {
        return s_instance.startIntakeImpl();
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

    private Command startIntakeImpl() {
        return Commands.parallel(
            m_winch.commandSetTargetAngleEnabled(() -> true, () -> Constants.WinchConstants.kIntakeAngle),
            m_swerve.commandSetTargetHeadingEnabled(() -> false),
            // m_shooter.commandStopShooterWheels(),
            m_rollers.commandStartIntakeProcedure()
        );
    }
}
