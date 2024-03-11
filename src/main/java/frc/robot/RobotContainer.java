// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Constants.DriveTeamConstants;
import frc.robot.commands.CentralCommandFactory;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(DriveTeamConstants.kDriverControllerPort);
  private final CommandJoystick m_operatorJoystick = new CommandJoystick(DriveTeamConstants.kOperatorJoystickPort);

  private final SlewRateLimiter m_driverXLimiter = new SlewRateLimiter(20);
  private final SlewRateLimiter m_driverYLimiter = new SlewRateLimiter(20);
  private final SlewRateLimiter m_driverRotLimiter = new SlewRateLimiter(10);

  private final SwerveSubsystem m_swerve;
  private final ShooterSubsystem m_shooter;
  private final IntakeSubsystem m_intake;
  private final ClimbSubsystem m_climb;


  private final SendableChooser<Command> m_autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    StateMachine.init();
    // Instantiate swerve
    m_swerve = new SwerveSubsystem();

    m_swerve.setDefaultCommand(
      m_swerve.getDriveWithJoystickCommand(
        () -> m_driverXLimiter.calculate(m_driverController.getLeftX()), // l/r
        () -> m_driverYLimiter.calculate(-m_driverController.getLeftY()), // f/b
        () -> m_driverRotLimiter.calculate(-m_driverController.getRightX()), // rot
        () -> m_driverController.getLeftTriggerAxis(), // super speed
        () -> m_driverController.getRightTriggerAxis(), // super slow
        () -> true // Field relative
      )
    );

    // TODO: change intake motor port
    m_intake = new IntakeSubsystem();

    m_shooter = new ShooterSubsystem(() -> m_swerve.getPose());

    m_climb = new ClimbSubsystem();

    m_shooter.getHasNoteTrigger().onTrue(m_intake.getStopIntakeCommand());

    CentralCommandFactory.init(m_intake, m_shooter, m_swerve, m_climb);
    
    // Register all commands for auto
    NamedCommands.registerCommand("startIntake", m_intake.getStartIntakeCommand());
    NamedCommands.registerCommand("stopIntake", m_intake.getStopIntakeCommand());

    NamedCommands.registerCommand("startAutoAim", CentralCommandFactory.getStartAutoAimCommand());
    NamedCommands.registerCommand("stopAutoAim", CentralCommandFactory.getStopAutoAimCommand());

    NamedCommands.registerCommand("startShoot", m_shooter.getStartFeederCommand());
    NamedCommands.registerCommand("stopShoot", m_shooter.getStopFeederCommand());

    // Get auto chooser
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("autoChooser", m_autoChooser);

    

    // Configure the controller bindings
    configureButtonBindings();
  }

  /** 
   * Configure button bindings for controllers (axis bindings may not be handled by this method)
  */
  private void configureButtonBindings() {
    m_driverController.a().onTrue(m_shooter.getToggleFeederCommand());
    m_driverController.b().onTrue(m_shooter.getToggleShooterCommand());
    m_driverController.x().onTrue(Commands.parallel(m_shooter.getToggleAutoAimCommand(), m_swerve.getToggleAutoAimCommand()));
//    m_driverController.y().onTrue(m_centralCommandFactory.getReverseAllCommand());
    m_driverController.leftBumper().onTrue(CentralCommandFactory.getToggleIntakeAndFeederCommand());
    m_driverController.rightBumper().onTrue(m_shooter.getAimAmpCommand());

    m_operatorJoystick.button(5).whileTrue(m_shooter.getIncreaseAngleCommand());
    m_operatorJoystick.button(4).whileTrue(m_shooter.getDecreaseAngleCommand());
    m_operatorJoystick.button(1).onTrue(m_shooter.getToggleShootAmp());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Get value of auto chooser
    return m_autoChooser.getSelected();
    // return null;
  }

}