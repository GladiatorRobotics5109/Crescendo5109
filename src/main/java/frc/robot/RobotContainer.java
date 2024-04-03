// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Constants.DriveTeamConstants;
import frc.robot.commands.AutonFactory;
import frc.robot.commands.CentralCommandFactory;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem.ModuleOrientation;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.logging.Loggable;
import frc.robot.util.logging.LoggableBoolean;
import frc.robot.util.logging.LoggableDouble;
import frc.robot.util.logging.Logger;

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
  //private final ClimbSubsystem m_climb;
  private final CentralCommandFactory m_centralCommandFactory;

  private final SendableChooser<Command> m_autoChooser;

  private final PowerDistribution m_pdp;

  //Shooter motors
  private final LoggableDouble m_leftShooterCurrentLog;
  private final LoggableDouble m_rightShooterCurrentLog;

  //Feeder motor
  private final LoggableDouble m_feederCurrentLog;

  //Winch motor
  private final LoggableDouble m_winchCurrentLog;

  //Intake Motor
  private final LoggableDouble m_intakeCurrentLog;

  //Climb motors
  private final LoggableDouble m_leftClimbCurrentLog;
  private final LoggableDouble m_rightClimbCurrentLog;

  // Swerve motors
  private final LoggableDouble m_frontLeftDriveCurrentLog;
  private final LoggableDouble m_frontLeftTurnCurrentLog;
  private final LoggableDouble m_frontRightDriveCurrentLog;
  private final LoggableDouble m_frontRightTurnCurrentLog;
  private final LoggableDouble m_backLeftDriveCurrentLog;
  private final LoggableDouble m_backLeftTurnCurrentLog;
  private final LoggableDouble m_backRightDriveCurrentLog;
  private final LoggableDouble m_backRightTurnCurrentLog;

  //Buckboost + Servo Power module
  private final LoggableDouble m_buckBoostAndServosCurrentLog;



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
        () -> m_driverController.getRightTriggerAxis() < 0.5 // Field relative
      )
    );

    // TODO: change intake motor port
    m_intake = new IntakeSubsystem();

    m_shooter = new ShooterSubsystem(() -> m_swerve.getPose());
    //m_climb = new ClimbSubsystem();
    m_shooter.getHasNoteTrigger().onTrue(m_intake.getStopIntakeCommand());
    m_shooter.getHasNoteTrigger().onFalse(m_swerve.getStopAutoAimCommand());

    m_centralCommandFactory = new CentralCommandFactory(m_intake, m_shooter, m_swerve);
    
    // Register all commands for auto
    NamedCommands.registerCommand("startIntake", m_intake.getStartIntakeCommand());
    NamedCommands.registerCommand("stopIntake", m_intake.getStopIntakeCommand());

    NamedCommands.registerCommand("startAutoAim", m_centralCommandFactory.getStartAutoAimCommand());
    NamedCommands.registerCommand("stopAutoAim", m_centralCommandFactory.getStopAutoAimCommand());

    NamedCommands.registerCommand("startFeed", m_shooter.getStartFeederCommand());
    NamedCommands.registerCommand("stopFeed", m_shooter.getStopFeederCommand());

    // Get auto chooser
    // m_autoChooser = AutoBuilder.buildAutoChooser();
    m_autoChooser = new SendableChooser<Command>();
    m_autoChooser.setDefaultOption("None", Commands.none());
    m_autoChooser.addOption("None", Commands.none());
    m_autoChooser.addOption("Taxi", AutonFactory.getTaxiCommand(m_swerve));
    m_autoChooser.addOption("ShootAndTaxi", AutonFactory.getShootAndTaxiCommand(m_swerve, m_shooter));
    m_autoChooser.addOption("WheelRadiusCharacterization", 
        Commands.sequence(
            m_swerve.getOrientModulesCommand(ModuleOrientation.CIRCLE),
            new WheelRadiusCharacterization(m_swerve, () -> m_swerve.getHeading().getRadians())
          )
        );

    m_autoChooser.addOption("ShooterFeedforwardCharacterization", AutonFactory.getShooterFeedforwardCharacterizationCommand(m_shooter));
    SmartDashboard.putData("autoChooser", m_autoChooser);



    // Configure the controller bindings
    configureButtonBindings();

    m_pdp = new PowerDistribution();

    //Shooter Motors
    m_leftShooterCurrentLog = new LoggableDouble("Left Shooter Current", true, true, () -> m_pdp.getCurrent(11));
    m_rightShooterCurrentLog = new LoggableDouble("Right Shooter Current", true, true, () -> m_pdp.getCurrent(10));

    //Feeder motor
    m_feederCurrentLog = new LoggableDouble("Feeder Current", true, true, () -> m_pdp.getCurrent(4));

    //Intake motor
    m_intakeCurrentLog = new LoggableDouble("Feeder Current", true, true, () -> m_pdp.getCurrent(7));

    //Winch motor
    m_winchCurrentLog =  new LoggableDouble("Winch Current", true, true, () -> m_pdp.getCurrent(5));

    //Climb motors
    m_leftClimbCurrentLog = new LoggableDouble("Left Climb Current", true, true, () -> m_pdp.getCurrent(8));
    m_rightClimbCurrentLog = new LoggableDouble("Right Climb Current", true, true, () -> m_pdp.getCurrent(9));

    // Swerve motors
    m_frontLeftDriveCurrentLog = new LoggableDouble("Front Left Drive Current", true, true, () -> m_pdp.getCurrent(15));
    m_frontLeftTurnCurrentLog = new LoggableDouble("Front Left Turn Current", true, true, () -> m_pdp.getCurrent(0));
    m_frontRightDriveCurrentLog = new LoggableDouble("Front Right Drive Current", true, true, () -> m_pdp.getCurrent(13));
    m_frontRightTurnCurrentLog = new LoggableDouble("Front Right Turn Current", true, true, () -> m_pdp.getCurrent(12));
    m_backLeftDriveCurrentLog = new LoggableDouble("Back Left Drive Current", true, true, () -> m_pdp.getCurrent(14));
    m_backLeftTurnCurrentLog = new LoggableDouble("Back Left Turn Current", true, true, () -> m_pdp.getCurrent(1));
    m_backRightDriveCurrentLog = new LoggableDouble("Back Right Drive Current", true, true, () -> m_pdp.getCurrent(2)); 
    m_backRightTurnCurrentLog = new LoggableDouble("Back Right Turn Current", true, true, () -> m_pdp.getCurrent(3));

    //Buckboost + Servo Power module
    m_buckBoostAndServosCurrentLog = new LoggableDouble("BuckBoost+Servos Current", true, true, () -> m_pdp.getCurrent(6));

    Logger.addLoggable(m_leftShooterCurrentLog);
    Logger.addLoggable(m_rightShooterCurrentLog);

    
    //CameraServer.startAutomaticCapture();
  }

  /** 
   * Configure button bindings for controllers (axis bindings may not be handled by this method)
  */
  private void configureButtonBindings() {
    m_driverController.a().onTrue(m_shooter.getToggleFeederCommand());
    m_driverController.b().onTrue(m_shooter.getToggleShooterCommand());
    m_driverController.x().onTrue(m_centralCommandFactory.getToggleAutoAimCommand());
    m_driverController.y().onTrue(m_shooter.getSetAngleCommand(30));
    m_driverController.leftBumper().onTrue(m_centralCommandFactory.getToggleIntakeAndFeederCommand());
    m_driverController.rightBumper().onTrue(m_shooter.getAimAmpCommand());

    m_operatorJoystick.button(1).onTrue(m_shooter.getToggleShooterCommand());
    m_operatorJoystick.button(2).onTrue(m_shooter.getToggleShootAmp());
    // m_operatorJoystick.button(2).onTrue(m_shooter.getStartFeederSlowCommand()).onFalse(m_shooter.getStopFeederCommand());
    m_operatorJoystick.button(3).onTrue(m_shooter.getReverseFeederSlowCommand()).onFalse(m_shooter.getStopFeederCommand());
    m_operatorJoystick.button(4).whileTrue(m_shooter.getDecreaseAngleCommand());
    m_operatorJoystick.button(5).whileTrue(m_shooter.getIncreaseAngleCommand());
    m_operatorJoystick.button(6).onTrue(m_shooter.getResetEncoderMaxCommand());
    m_operatorJoystick.button(7).onTrue(m_shooter.getResetEncoderMinCommand());
    m_operatorJoystick.button(8).onTrue(m_shooter.getSetOverrideMinMaxAngleCommand(true)).onFalse(m_shooter.getSetOverrideMinMaxAngleCommand(false));
    m_operatorJoystick.button(9).onTrue(m_shooter.getToggleBarCommand());
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