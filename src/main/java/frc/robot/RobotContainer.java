// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.rollers.Rollers;
import frc.robot.util.Util;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.winch.WinchSubsystem;

public class RobotContainer {
    private final VisionSubsystem m_vision;
    private final SwerveSubsystem m_swerve;
    private final WinchSubsystem m_winch;
    private final ShooterSubsystem m_shooter;
    private final Rollers m_rollers;

    private CommandPS5Controller m_driverController;

    private LoggedDashboardChooser<Command> m_autoChooser;

    public RobotContainer() {
        m_vision = new VisionSubsystem();
        m_swerve = new SwerveSubsystem();
        m_winch = new WinchSubsystem();
        m_shooter = new ShooterSubsystem();
        m_rollers = new Rollers();

        m_driverController = new CommandPS5Controller(Constants.DriveTeamConstants.kDriverControllerPort);

        StateMachine.init(m_swerve, m_vision, m_shooter, m_winch, m_rollers);
        CommandBuilder.init(m_swerve, m_vision, m_shooter, m_winch, m_rollers, m_driverController);

        // m_rollers.getFeederHasNoteTrigger().onTrue(CommandBuilder.commandDriverControllerNoteEnterSequence());

        configureBindings();
        registerNamedCommands();

        m_autoChooser = new LoggedDashboardChooser<Command>("AutoChooser");

        m_autoChooser.addDefaultOption("DoNothing", AutoBuilder.doNothing(m_swerve));
        m_autoChooser.addOption("Test", AutoBuilder.test(m_swerve));

        CommandScheduler.getInstance().onCommandInitialize(
            (Command command) -> System.out.println(
                "Command Started:\n    Name: " + command.getName() + "\n    Subsystem: " + command.getSubsystem()
            )
        );

        CommandScheduler.getInstance().onCommandFinish(
            (Command command) -> System.out.println(
                "Command Finished:\n    Name: " + command.getName() + "\n    Subsystem: " + command.getSubsystem()
            )
        );
    }

    public Command commandGetAutonomous() {
        return m_autoChooser.get();
    }

    private void configureBindings() {
        m_swerve.setDefaultCommand(
            m_swerve.commandDriveWithJoystick(
                m_driverController::getLeftX,
                m_driverController::getLeftY,
                m_driverController::getRightX,
                () -> Constants.TeleopConstants.kDriveFieldRelative
            )
        );

        m_driverController.square().onTrue(CommandBuilder.commandToggleAutoAim());
        m_driverController.cross().onTrue(CommandBuilder.commandToggleManualShoot());
        m_driverController.circle().onTrue(CommandBuilder.commandDriverControllerNoteEnterSequence());
        m_driverController.L1().onTrue(CommandBuilder.commandToggleIntake());
        m_driverController.R1().onTrue(CommandBuilder.commandToggleAssistedShoot());
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("PrintHello", Commands.print("HELLO WORLD"));

        NamedCommands.registerCommand(
            "Aim",
            Commands.sequence(
                m_swerve.commandSetTargetHeadingEnabled(true, Util::targetHeadingTest),
                Commands.print("AIMING")
            )
        );

        NamedCommands.registerCommand(
            "Shoot",
            Commands.sequence(
                Commands.print("WAITING UNTIL AIM GOOD"),
                Commands.waitUntil(m_swerve::isAtTargetHeading),
                m_swerve.commandSetTargetHeadingEnabled(false),
                Commands.print("SHOOT")
            )
        );
    }
}
