// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveTeamConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final CommandXboxController m_driverController =
      new CommandXboxController(DriveTeamConstants.kDriverControllerPort);

  private final SwerveSubsystem m_swerve;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the controller bindings
    configureButtonBindings();

    // instantiate swerve
    m_swerve = new SwerveSubsystem();

    m_swerve.setDefaultCommand(
      m_swerve.getDriveWithJoystickCommand(
        () -> m_driverController.getLeftX(), // l/r
        () -> m_driverController.getLeftY(), // f/b
        () -> m_driverController.getRightX(), // rot
        () -> true) // field relative
    );
  }

  /** 
   * Configure button bindings for controllers (axis bindings may not be handled by this method)
  */
  private void configureButtonBindings() {
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
