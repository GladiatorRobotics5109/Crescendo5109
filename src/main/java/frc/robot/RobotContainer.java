// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RobotContainer {
    private SwerveSubsystem m_swerve;

    private CommandPS5Controller m_driverController;

    // private GenericHID m_driverController;

    public RobotContainer() {
        m_swerve = new SwerveSubsystem();

        m_driverController = new CommandPS5Controller(Constants.DriveTeamConstants.kDriverControllerPort);

        configureBindings();
    }

    private void configureBindings() {
        m_swerve.setDefaultCommand(
            m_swerve.driveWithJoystickCommand(
                m_driverController::getLeftX,
                m_driverController::getLeftY,
                m_driverController::getRightX,
                () -> Constants.TeleopConstants.kFieldRelative
            )
        );

        // m_swerve.setDefaultCommand(
        // m_swerve.driveWithJoystickCommand(
        // () -> m_driverController.getRawAxis(0),
        // () -> m_driverController.getRawAxis(1),
        // () -> m_driverController.getRawAxis(2),
        // () -> Constants.TeleopConstants.kFieldRelative
        // )
        // );
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
