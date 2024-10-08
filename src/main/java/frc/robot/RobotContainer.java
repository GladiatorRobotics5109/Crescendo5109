// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Constants.DriveTeamConstants;
import frc.robot.commands.AutonFactory;
import frc.robot.commands.CentralCommandFactory;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
  // private final ClimbSubsystem m_climb;
  private final CentralCommandFactory m_centralCommandFactory;

  private final SendableChooser<Command> m_autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    StateMachine.init();
    // Instantiate the swerve subsystem
    m_swerve = new SwerveSubsystem();

    // Set the default command on the swerve subsystem to be the drive with joystick command
    m_swerve.setDefaultCommand(
      m_swerve.getDriveWithJoystickCommand(
        () -> m_driverXLimiter.calculate(-m_driverController.getLeftX()), // l/r
        () -> m_driverYLimiter.calculate(-m_driverController.getLeftY()), // f/b
        () -> m_driverRotLimiter.calculate(-m_driverController.getRightX()), // rot
        () -> m_driverController.getLeftTriggerAxis(), // super speed
        () -> m_driverController.getRightTriggerAxis(), // super slow
        () -> true // Field relative
      )
    );

    // Instantiate intake subsystem
    m_intake = new IntakeSubsystem();

    // Instantiate shooter subsystem
    m_shooter = new ShooterSubsystem(() -> m_swerve.getPose());

    // Instantiate climb subsystem
    // m_climb = new ClimbSubsystem();

    // Bind has not trigger to stopping intake and auto aim
    m_shooter.getHasNoteTrigger().onTrue(m_intake.getStopIntakeCommand());
    m_shooter.getHasNoteTrigger().onFalse(m_swerve.getStopAutoAimCommand());

    // Create the centeral command factory
    m_centralCommandFactory = new CentralCommandFactory(m_intake, m_shooter, m_swerve);
    
    // Register all commands used in auto
    NamedCommands.registerCommand("startIntake", m_centralCommandFactory.getStartIntakeAndFeederCommand());
    NamedCommands.registerCommand("stopIntake", m_centralCommandFactory.getStopIntakeAndFeederCommand());

    NamedCommands.registerCommand("startAutoAim", m_centralCommandFactory.getStartAutoAimCommand());
    NamedCommands.registerCommand("stopAutoAim", m_centralCommandFactory.getStopAutoAimCommand());

    NamedCommands.registerCommand("startShooter", m_shooter.getStartShooterCommand());
    NamedCommands.registerCommand("stopShooter", m_shooter.getStopShooterCommand());

    NamedCommands.registerCommand("startFeed", m_shooter.getStartFeederCommand());
    NamedCommands.registerCommand("stopFeed", m_shooter.getStopFeederCommand());

    NamedCommands.registerCommand("waitForNoteEnter", m_shooter.getWaitForNoteEnterCommand());
    NamedCommands.registerCommand("waitForNoteExit", m_shooter.getWaitForNoteExitCommand());

    // Get auto chooser
    // m_autoChooser = AutoBuilder.buildAutoChooser();
    m_autoChooser = new SendableChooser<Command>();
    m_autoChooser.setDefaultOption("None", Commands.none());
    m_autoChooser.addOption("None", Commands.none());
    // -- BASIC AUTOS --
    m_autoChooser.addOption("OldTaxi", AutonFactory.getTaxiCommand(m_swerve));
    m_autoChooser.addOption("OldShootAndTaxi", AutonFactory.getShootAndTaxiCommand(m_swerve, m_shooter));
   
    // -- PATH PLANNER AUTOS -- 
    m_autoChooser.addOption("R11S", AutoBuilder.buildAuto("R11S"));
    m_autoChooser.addOption("R11ST", AutoBuilder.buildAuto("R11ST"));
    m_autoChooser.addOption("R21S", AutoBuilder.buildAuto("R21S"));
    m_autoChooser.addOption("R21ST", AutoBuilder.buildAuto("R21ST"));
    m_autoChooser.addOption("R31S", AutoBuilder.buildAuto("R31S"));
    m_autoChooser.addOption("R31ST", AutoBuilder.buildAuto("R31ST"));
    m_autoChooser.addOption("R34S", AutoBuilder.buildAuto("R34S"));
    m_autoChooser.addOption("B1PushNote1", AutoBuilder.buildAuto("B1PushNote1"));
    m_autoChooser.addOption("B1PushNote2", AutoBuilder.buildAuto("B1PushNote2"));
    m_autoChooser.addOption("B11S", AutoBuilder.buildAuto("B11S"));
    m_autoChooser.addOption("B11ST", AutoBuilder.buildAuto("B11ST"));
    m_autoChooser.addOption("B12ST", AutoBuilder.buildAuto("B12ST"));
    m_autoChooser.addOption("B21S", AutoBuilder.buildAuto("B21S"));
    m_autoChooser.addOption("B31S", AutoBuilder.buildAuto("B31S"));
    m_autoChooser.addOption("B31ST", AutoBuilder.buildAuto("B31ST"));
    m_autoChooser.addOption("B34S", AutoBuilder.buildAuto("B34S"));
   
    // -- TEST AUTOS --
    m_autoChooser.addOption("Test", AutoBuilder.buildAuto("Test"));
    m_autoChooser.addOption("WaitForNoteEnter", Commands.sequence(
      Commands.print("START INTAKE + FEED"),
      m_centralCommandFactory.getStartIntakeAndFeederCommand(),
      Commands.print("WAIT"),
      m_shooter.getWaitForNoteEnterCommand(),
      Commands.print("STOP INTAKE + FEED"),
      m_centralCommandFactory.getStopIntakeAndFeederCommand()
    ));
    m_autoChooser.addOption("SwerveModuleTurn SysId", m_swerve.getModuleTurnSysIdCommand());

    SmartDashboard.putData("autoChooser", m_autoChooser);

    // Configure the controller bindings
    configureButtonBindings();

    //CameraServer.startAutomaticCapture();
  }

  /** 
   * Configure button bindings for controllers (axis bindings may not be handled by this method)
  */
  private void configureButtonBindings() {
    m_driverController.a().onTrue(m_shooter.getToggleFeederCommand());
    m_driverController.b().onTrue(m_shooter.getToggleShooterCommand());
    m_driverController.x().onTrue(m_centralCommandFactory.getToggleAutoAimCommand());
    m_driverController.y().onTrue(m_shooter.getSetAngleCommand(35));
    m_driverController.leftBumper().onTrue(m_centralCommandFactory.getToggleIntakeAndFeederCommand());
    m_driverController.rightBumper().onTrue(m_shooter.getAimAmpCommand());

    m_operatorJoystick.button(1).onTrue(m_shooter.getToggleBarCommand());
    m_operatorJoystick.button(2).onTrue(m_shooter.getToggleShootAmp());
    m_operatorJoystick.button(4).whileTrue(m_shooter.getDecreaseAngleCommand());
    m_operatorJoystick.button(5).whileTrue(m_shooter.getIncreaseAngleCommand());
    m_operatorJoystick.button(6).onTrue(m_shooter.getResetEncoderMaxCommand());
    // m_operatorJoystick.button(6).onTrue(m_climb.getRetractLeftCommand()).onFalse(m_climb.getStopLeftCommand());
    // m_operatorJoystick.button(7).whileTrue(m_climb.getExtendLeftCommand()).onFalse(m_climb.getStopLeftCommand());
    m_operatorJoystick.button(8).onTrue(m_shooter.getSetOverrideMinMaxAngleCommand(true)).onFalse(m_shooter.getSetOverrideMinMaxAngleCommand(false));
    m_operatorJoystick.button(9).onTrue(m_shooter.getResetEncoderMinCommand()); 
    // m_operatorJoystick.button(10).onTrue(m_climb.getRetractRightCommand()).onFalse(m_climb.getStopRightCommand());
    // m_operatorJoystick.button(11).onTrue(m_climb.getExtendRightCommand()).onFalse(m_climb.getStopRightCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Get value of auto chooser
    return Commands.sequence(
      m_swerve.getResetPoseAllianceCommand(),
      //m_climb.getRetractCommand(),
      // m_swerve.getAlignWheelCommand(),
      m_autoChooser.getSelected()
    );
    // return null;
  }
}