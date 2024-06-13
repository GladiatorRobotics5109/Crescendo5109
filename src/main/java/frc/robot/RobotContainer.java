// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
    private VisionSubsystem m_vision;
    private SwerveSubsystem m_swerve;

    private CommandPS5Controller m_driverController;
    // private GenericHID m_driverController;

    public RobotContainer() {
        m_vision = new VisionSubsystem();
        m_swerve = new SwerveSubsystem();

        StateMachine.init(m_vision, m_swerve);

        m_driverController = new CommandPS5Controller(Constants.DriveTeamConstants.kDriverControllerPort);
        // m_driverController = new GenericHID(0);

        configureBindings();
    }

    private void configureBindings() {
        m_swerve.setDefaultCommand(
            m_swerve.driveWithJoystickCommand(
                m_driverController::getLeftX,
                m_driverController::getLeftY,
                m_driverController::getRightX,
                () -> Constants.TeleopConstants.kDriveFieldRelative
            )
        );

        // m_swerve.setDefaultCommand(
        // m_swerve.driveWithJoystickCommand(
        // () -> m_driverController.getRawAxis(0),
        // () -> m_driverController.getRawAxis(1),
        // () -> m_driverController.getRawAxis(2),
        // () -> Constants.TeleopConstants.kDriveFieldRelative
        // )
        // );
    }

    public Command getAutonomousCommand() {
        return AutoBuilder.test(m_swerve);
    }
}
