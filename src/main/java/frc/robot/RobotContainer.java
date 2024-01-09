// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Auton.Autos;
import frc.robot.Constants.DriveTeamConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.VisionManager;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandXboxController m_driverController =
      new CommandXboxController(DriveTeamConstants.kDriverControllerPort);

  private final SlewRateLimiter m_driverXLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter m_driverYLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter m_driverRotLimiter = new SlewRateLimiter(5);

  private final SwerveSubsystem m_swerve;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Autos.init();
    
    // try {
    //   Common.currentAprilTagFieldLayout = AprilTagFieldLayout.loadFromResource("apriltagFieldLayouts/layout.json");
    // }
    // catch (IOException e) {
    //   DriverStation.reportError("Unable to open apriltag field layout!", e.getStackTrace());
    //   Common.currentAprilTagFieldLayout = null;
    // }

    VisionManager.init();

    // instantiate swerve
    m_swerve = new SwerveSubsystem();

    m_swerve.setDefaultCommand(
      m_swerve.getDriveWithJoystickCommand(
        () -> m_driverXLimiter.calculate(m_driverController.getLeftX()), // l/r
        () -> m_driverYLimiter.calculate(-m_driverController.getLeftY()), // f/b
        () -> m_driverRotLimiter.calculate(-m_driverController.getRightX()), // rot
        () -> Constants.SwerveConstants.kFieldRelative) // field relative
    );

    
    // Configure the controller bindings
    configureButtonBindings();
  }

  /** 
   * Configure button bindings for controllers (axis bindings may not be handled by this method)
   */
  private void configureButtonBindings() {
    m_driverController.a().onTrue(m_swerve.getAlignWheelCommand());
    m_driverController.b().whileTrue(m_swerve.getBrakeAndXCommand());
    m_driverController.y().onTrue(m_swerve.getCoastAllCommand());
    
    // slow mode/fast mode
    m_driverController.rightTrigger().whileTrue(m_swerve.getSuperSpeedCommand(() -> m_driverController.getLeftTriggerAxis()));
    m_driverController.rightBumper().onTrue(m_swerve.getSetSlowModeCommand(() -> true))
                                    .onFalse(m_swerve.getSetSlowModeCommand(() -> false));
    // m_driverController.x().onTrue(m_swerve.getDriveWithTrajectoryCommand(() -> Timer.getFPGATimestamp(), Common.currentAutonTrajectory));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return null for now not to run auton command
    return null;
    // return Autos.getCurrentAutoCommand();
  }

  public void resetSwerveEncoders() {
    m_swerve.resetTurnEncoders();
  }
}
