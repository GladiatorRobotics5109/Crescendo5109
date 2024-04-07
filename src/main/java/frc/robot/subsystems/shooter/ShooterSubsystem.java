package frc.robot.subsystems.shooter;

import java.sql.Driver;
import java.util.Optional;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.util.Constants.ShooterConstants;
import frc.robot.util.logging.LoggableBoolean;
import frc.robot.util.logging.LoggableDouble;
import frc.robot.stateMachine.ShooterState;
import frc.robot.stateMachine.StateMachine;
import frc.robot.stateMachine.ShooterState.ShooterStateEnum;
import frc.robot.util.logging.Logger;

/**
 * Represents the Shooter
 */
public class ShooterSubsystem extends SubsystemBase {
    
    private final CANSparkMax m_leftShooterMotor;  // Left Side Shooter Wheels
    private final CANSparkMax m_rightShooterMotor; // Right Side Shooter Wheels
    private final CANSparkMax m_feederMotor; // Feeder Wheels
    private final CANSparkMax m_winchMotor; // Shooter Tilt Winch
    private final Servo m_leftBarServo;
    private final Servo m_rightBarServo;

    private boolean m_overrideMinMaxAngle;

    private final SparkPIDController m_leftShooterPIDController;
    private final SparkPIDController m_rightShooterPIDController;
    private final SparkPIDController m_feederPIDController;
    private final SparkPIDController m_winchPIDController;
    // private final SparkPIDController m_barPIDController;

    private final RelativeEncoder m_leftShooterEncoder;
    private final RelativeEncoder m_rightShooterEncoder;
    private final RelativeEncoder m_feederEncoder;
    private final RelativeEncoder m_winchEncoder;
    // private final RelativeEncoder m_barEncoder;

    private final DigitalInput m_feederSensor;
    private final DigitalInput m_angleLimitSwitch;

    private double m_desiredAngle;


    private final Trigger m_feederSensorTrigger;
    private final Trigger m_debouncedFeederSensorTrigger;

    private final Trigger m_limitSwitchTrigger;

    private final Trigger m_reachedAngleSetpointTrigger;

    private Supplier<Pose2d> m_poseSupplier;

    private final ShooterState m_state;

    // private final LoggableDouble m_desiredRps;
    // private final LoggableDouble m_currentRps;

    private final LoggableDouble m_desiredAngleLog;
    private final LoggableDouble m_currentAngleLog;
    private final LoggableBoolean m_autoAimingLog;

    private final LoggableBoolean m_hasNoteLog;
    private final LoggableBoolean m_feederSensorLog;

    private final LoggableDouble m_rpmLLog;
    private final LoggableDouble m_rpmRLog;

    private final LoggableDouble m_rBusCurrent;
    private final LoggableDouble m_lBusCurrent;

    private final LoggableDouble m_rOutputCurrent;
    private final LoggableDouble m_lOutputCurrent;

    private final LoggableDouble m_winchOutputCurrentLog;

    public ShooterSubsystem(Supplier<Pose2d> poseSupplier) {
        m_state = StateMachine.getShooterState();

        m_leftShooterMotor = new CANSparkMax(ShooterConstants.kLeftShooterMotorPort, MotorType.kBrushless);
        m_rightShooterMotor = new CANSparkMax(ShooterConstants.kRightShooterMotorPort, MotorType.kBrushless);
        m_feederMotor = new CANSparkMax(ShooterConstants.kFeederMotorPort, MotorType.kBrushless);
        m_winchMotor = new CANSparkMax(ShooterConstants.kWinchMotorPort, MotorType.kBrushless);
        m_leftBarServo = new Servo(ShooterConstants.kLeftBarActuatorChannel);
        m_rightBarServo = new Servo(ShooterConstants.kRightBarActuatorChannel);


        //taken from example: yourActuator.setBounds(2.0, 1.8, 1.5, 1.2, 1.0) (assuming values are in ms in examlpe)
        m_leftBarServo.setBoundsMicroseconds(2500, 1504, 1500, 1496, 500);
        m_rightBarServo.setBoundsMicroseconds(500, 1504, 1500, 1496, 2500);
        // m_rightBarServo.setBoundsMicroseconds(2500, 1504, 1500, 1496, 500);

        m_leftShooterPIDController = m_leftShooterMotor.getPIDController();
        m_rightShooterPIDController = m_rightShooterMotor.getPIDController();

        m_feederPIDController = m_feederMotor.getPIDController();
        m_winchPIDController = m_winchMotor.getPIDController();

        m_leftShooterEncoder = m_leftShooterMotor.getEncoder();
        m_rightShooterEncoder = m_rightShooterMotor.getEncoder();
        m_feederEncoder = m_feederMotor.getEncoder();
        m_winchEncoder = m_winchMotor.getEncoder();

        m_leftShooterMotor.setIdleMode(IdleMode.kBrake);
        m_rightShooterMotor.setIdleMode(IdleMode.kBrake);
        m_feederMotor.setIdleMode(IdleMode.kBrake);
        m_winchMotor.setIdleMode(IdleMode.kBrake);

        m_leftShooterPIDController.setP(ShooterConstants.kShooterP);
        m_leftShooterPIDController.setI(ShooterConstants.kShooterI);
        m_leftShooterPIDController.setD(ShooterConstants.kShooterD);

        m_rightShooterPIDController.setP(ShooterConstants.kShooterP);
        m_rightShooterPIDController.setI(ShooterConstants.kShooterI);
        m_rightShooterPIDController.setD(ShooterConstants.kShooterD);

        m_feederPIDController.setP(ShooterConstants.kFeederP);
        m_feederPIDController.setI(ShooterConstants.kFeederI);
        m_feederPIDController.setIZone(5);
        m_feederPIDController.setD(ShooterConstants.kFeederD);

        m_winchPIDController.setP(ShooterConstants.kWinchP);
        m_winchPIDController.setI(ShooterConstants.kWinchI);
        m_winchPIDController.setD(ShooterConstants.kWinchD);
        m_winchPIDController.setFF(0.001);

        m_poseSupplier = poseSupplier;

        m_desiredAngleLog = new LoggableDouble("Shooter Desired Angle", true);
        m_currentAngleLog = new LoggableDouble("Shooter Current Angle", true);
        m_autoAimingLog = new LoggableBoolean("Shooter Auto Aiming", true);
        m_rpmLLog = new LoggableDouble("Shooter L RPM", true);
        m_rpmRLog = new LoggableDouble("Shooter R RPM", true);
        m_hasNoteLog = new LoggableBoolean("HasNote", true);
        m_feederSensorLog = new LoggableBoolean("Feeder Sensor", true);

        m_rBusCurrent = new LoggableDouble("Bus Current R", true, true, () -> m_rightShooterMotor.getBusVoltage());
        m_lBusCurrent = new LoggableDouble("Bus Current L", true, true, () -> m_leftShooterMotor.getBusVoltage());

        m_rOutputCurrent = new LoggableDouble("Output Current R", true, true, () -> m_rightShooterMotor.getOutputCurrent());
        m_lOutputCurrent = new LoggableDouble("Output Currenty L", true, true, () -> m_leftShooterMotor.getOutputCurrent());

        m_winchOutputCurrentLog = new LoggableDouble("Winch Output Current", true, true, () -> m_winchMotor.getOutputCurrent());

        Logger.addLoggable(m_rBusCurrent);
        Logger.addLoggable(m_lBusCurrent);
        Logger.addLoggable(m_rOutputCurrent);
        Logger.addLoggable(m_lOutputCurrent);
        Logger.addLoggable(m_winchOutputCurrentLog);

        m_overrideMinMaxAngle = false;

        // m_shooterPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        // m_feederPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        // m_winchPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        // m_barPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

        m_feederSensor = new DigitalInput(ShooterConstants.kFeederSensorChannel);
        m_angleLimitSwitch = new DigitalInput(ShooterConstants.kLimitSwitchChannel);

        m_winchEncoder.setPosition((50 - 57.8763) / (-1.07687));


        m_feederSensorTrigger = new Trigger(() -> m_feederSensor.get());
        m_limitSwitchTrigger = new Trigger(() -> m_angleLimitSwitch.get()).negate();

        //negated since the beambreak will return true until beam is broken

        // m_debouncedFeederSensorTrigger = m_feederSensorTrigger.debounce(0.01, DebounceType.kBoth).negate();
        m_debouncedFeederSensorTrigger = m_feederSensorTrigger.debounce(0.05, DebounceType.kBoth).negate();

        // m_debouncedFeederSensorTrigger = m_feederSensorTrigger.negate();

        m_desiredAngle = 52;
        m_reachedAngleSetpointTrigger = new Trigger(() -> atAngleSetpoint());


        configureBindings();
    }
    /**
     * These bindings make it so that when a note is in the shooter, the shooter wheels are spinning and ready to shoot
     * 
     */
    private void configureBindings() {
        m_debouncedFeederSensorTrigger.and(() -> DriverStation.isAutonomous() == false).onTrue(
            Commands.sequence(
                getAddHasNoteStateCommand(),
                getStopShooterCommand(),
                this.runOnce(() -> {
                    m_feederMotor.set(0.1);
                    m_leftShooterMotor.set(0.001);
                    m_rightShooterMotor.set(-0.001);
                }),
                Commands.waitSeconds(0.15),
                getStopFeederCommand(),
                getStopShooterCommand(),
                Commands.print("NOTE ENTER")
            )
        );

        // m_debouncedFeederSensorTrigger.onTrue(
        //     Commands.sequence(
        //         getAddHasNoteStateCommand(),
        //         getStopShooterCommand(),
        //         this.runOnce(() -> {
        //             m_feederMotor.set(0.1);
        //             m_leftShooterMotor.set(0.001);
        //             m_rightShooterMotor.set(-0.001);
        //         }),
        //         Commands.waitSeconds(0.15),
        //         getStopFeederCommand(),
        //         getStopShooterCommand(),
        //         Commands.print("NOTE ENTER")
        //     )
        // );

        m_debouncedFeederSensorTrigger.and(() -> DriverStation.isAutonomous() == false).onFalse(
            Commands.sequence(
                this.runOnce(() -> {m_state.removeState(ShooterStateEnum.HAS_NOTE);}),
                Commands.waitSeconds(0.5),
                // getStopShooterCommand(),
                Commands.runOnce(this::stopShooter, this),
                getStopFeederCommand(),
                Commands.print("NOTE EXIT")
            )
        );

        // m_limitSwitchTrigger.onTrue(Commands.run(() -> {
        // m_winchEncoder.setPosition((50 - 57.8763) / (-1.07687));
        // setAngle(50);
        // }));
    }

    public Command getAimAmpCommand() {
        return this.runOnce(() -> {
            setAngle(59);
            setBarExtension(0);
        }); 
    }


    public Command getStartShooterCommand() {
        return this.runOnce(() -> {
            startShooter();
        });
    }

    public Command getStopShooterCommand() {
        return this.runOnce(() -> {
            stopShooter();
        });
    }

    public Command getToggleShooterCommand() {
        return this.runOnce(() -> {
            toggleShooter();
        });
    }

    public Command getStartFeederCommand() {
        return this.runOnce(() -> {
            System.out.println("Start feed");
            startFeeder();
        });
    }

    public Command getStartFeederSlowCommand() {
        return this.runOnce(() -> {
            startFeederSlow();
        });
    }

    public Command getReverseFeederSlowCommand() {
        return this.runOnce(() -> {
            reverseFeederSlow();
        });
    }

    public Command getStopFeederCommand() {
        return this.runOnce(() -> {
            stopFeeder();
        });
    }

    public Command getToggleFeederCommand() {
        return this.runOnce(() -> {
            toggleFeeder();
        });
    }

    public Command getAddHasNoteStateCommand() {
        return this.runOnce(() -> {
            m_state.addState(ShooterStateEnum.HAS_NOTE);
        });
    }

    public Command getRemoveHasNoteStateCommand() {
        return this.runOnce(() -> {
            m_state.removeState(ShooterStateEnum.HAS_NOTE);
        });
    }

    public Command getSetAngleCommand(double angle) {
        return this.runOnce(() -> {
            m_state.removeState(ShooterStateEnum.AUTO_AIMING);
            setAngle(angle);
        });
    }

    public Command getToggleReverseBothCommand() {
        return this.runOnce(() -> {
            toggleReverseBoth();
        });
    }

    public Command getToggleAutoAimCommand() {
        return this.runOnce(() -> toggleAutoAim());
    }

    public Command getStartAutoAimCommand() {
        return this.runOnce(() -> startAutoAim());
    }

    public Command getStopAutoAimCommand() {
        return this.runOnce(() -> stopAutoAim());
    }

    public Command getIncreaseAngleCommand() {
        return this.run(() -> increaseAngle());
    }

    public Command getDecreaseAngleCommand() {
        return this.run(() -> decreaseAngle());
    }

    public Command getStartShootAmpCommand() {
        return this.runOnce(() -> startShootAmp());
    }

    public Command getToggleShootAmp() {
        return this.runOnce(() -> toggleShootAmp());
    }

    public Command getAutoAimAndShootCommand() {
        return this.run(() -> {
            setAngle(calcAutoAim());

            if (atAngleSetpoint()) {
                startFeeder();
            }
        });
    }

    public Command getResetEncoderMinCommand() {
        return this.runOnce(() -> {
            m_winchEncoder.setPosition((30 - 57.8763) / (-1.07687));
            setAngle(30);
        });
    }

    public Command getResetEncoderMaxCommand() {
        return this.runOnce(() -> {
            m_winchEncoder.setPosition((52 - 57.8763) / (-1.07687));
            setAngle(52);
        });
    }



    public Command getSetOverrideMinMaxAngleCommand(boolean value) {
        return this.runOnce(() -> m_overrideMinMaxAngle = value);
    }

    public Command getExtendBarCommand() {
        return this.runOnce(() -> swingBar());
    }

    public Command getResetBarCommand() {
        return this.runOnce(() -> resetBar());
    }

    public Command getToggleBarCommand() {
        return this.runOnce(() -> {
            if (m_state.is(ShooterStateEnum.BAR_EXTENDED)) {
                resetBar();
            } else {
                swingBar();
            }
        });
    }

    public Command getWaitForNoteEnterCommand() {
        return Commands.waitUntil(() -> m_debouncedFeederSensorTrigger.getAsBoolean() == true).andThen(
            Commands.sequence(
                Commands.print("    STATE"),
                // this.runOnce(() -> {m_state.addState(ShooterStateEnum.HAS_NOTE);}),
                Commands.print("    STOP SHOOTER"),
                getStopShooterCommand(),
                Commands.print("    REVERSE REVERSE"),
                this.runOnce(() -> {
                    m_feederMotor.set(0.1);
                    m_leftShooterMotor.set(0.001);
                    m_rightShooterMotor.set(-0.001);
                }),
                Commands.print("    WAIT SEC"),
                Commands.waitSeconds(0.15),
                Commands.print("    STOP FEED"),
                getStopFeederCommand(),
                Commands.print("   STOP SHOOTER"),
                getStopShooterCommand(),
                Commands.print("NOTE ENTER")
            )
        );
    }

    public Command getWaitForNoteExitCommand() {
        return Commands.waitUntil(() -> m_debouncedFeederSensorTrigger.getAsBoolean() == false).andThen(
            Commands.sequence(
                Commands.print("WAIT NOTE EXIT"),
                this.runOnce(() -> {m_state.removeState(ShooterStateEnum.HAS_NOTE);}),
                Commands.waitSeconds(0.5),
                // getStopShooterCommand(),
                Commands.runOnce(this::stopShooter, this),
                getStopFeederCommand(),
                Commands.print("NOTE EXIT")
            )
        );
    }



    public void swingBar() {
        m_leftBarServo.set(m_leftBarServo.getAngle() + 50);
        m_rightBarServo.set(m_rightBarServo.getAngle() + 50);
        m_state.addState(ShooterStateEnum.BAR_EXTENDED);
    }

    public void setLimitSwitchAngle() {
        m_winchEncoder.setPosition((50 - 57.8763) / (-1.07687));
    }

    public void resetBar() {
        m_leftBarServo.set(m_leftBarServo.getAngle() - 50);
        m_rightBarServo.set(m_rightBarServo.getAngle() - 50);
        m_state.removeState(ShooterStateEnum.BAR_EXTENDED);
    }

    public void setAngle(double angle) {
        if (!m_overrideMinMaxAngle && angle > 59) {
            System.out.println("MAX: " + angle);
            return;
        }
        if (!m_overrideMinMaxAngle && angle < 30) {
            System.out.println("MIN: " + angle);
            return;
        }

        m_desiredAngle = angle;

        double desiredRot = (angle - 57.8763) / (-1.07687);

        m_desiredAngleLog.log(Units.degreesToRadians(angle));

        m_winchPIDController.setReference(desiredRot, ControlType.kPosition);
    }

    //TODO: Figure out optimal RPM for shooter
    public void startShooter() {
        // m_desiredRps.log(2000.0);
        //m_leftShooterPIDController.setReference(5000, ControlType.kVelocity);
        //m_rightShooterPIDController.setReference(5000, ControlType.kVelocity);
        m_leftShooterMotor.set(-1);
        m_rightShooterMotor.set(1);
        m_state.addState(ShooterStateEnum.SHOOTER_WHEEL_SPINNING);
    }

    public void stopShooter() {
        //m_leftShooterPIDController.setReference(0, ControlType.kVelocity);
        //m_rightShooterPIDController.setReference(0, ControlType.kVelocity);
        m_leftShooterMotor.set(0);
        m_rightShooterMotor.set(0);
        m_state.removeState(ShooterStateEnum.SHOOTER_WHEEL_SPINNING);
    }

    public void toggleShooter() {
        if (m_state.is(ShooterStateEnum.SHOOTER_WHEEL_SPINNING)) {
            stopShooter();
        }
        else {
            startShooter();
        }
    }

    public void startShootAmp() {
        m_leftShooterMotor.setVoltage(-1.3);
        m_rightShooterMotor.setVoltage(1.3);
        // m_leftShooterMotor.set(-0.24);
        // m_rightShooterMotor.set(0.24);
        m_state.addState(ShooterStateEnum.SHOOTER_WHEEL_SPINNING);
    }

    public void toggleShootAmp() {
        if (m_state.is(ShooterStateEnum.SHOOTER_WHEEL_SPINNING)) {
            stopShooter();
        }
        else {
            startShootAmp();
        }
    }

    //TODO: Figure out optimal RPM for feeder
    public void startFeeder() {
        //m_feederPIDController.setReference(500, ControlType.kVelocity);
        m_feederMotor.set(-1);
        m_state.addState(ShooterStateEnum.FEEDER_WHEELS_SPINNING);
    }

    public void startFeederSlow() {
        m_feederMotor.set(0.2);
        m_state.addState(ShooterStateEnum.FEEDER_WHEELS_SPINNING);
    }

    public void reverseFeederSlow() {
        m_feederMotor.set(-0.2);
        m_state.addState(ShooterStateEnum.FEEDER_WHEELS_SPINNING);
    }

    public void stopFeeder() {
        //m_feederPIDController.setReference(0, ControlType.kVelocity);
        m_feederMotor.set(0);
        m_state.removeState(ShooterStateEnum.FEEDER_WHEELS_SPINNING);
    }

    public void toggleFeeder() {
        if (m_state.is(ShooterStateEnum.FEEDER_WHEELS_SPINNING)) {
            stopFeeder();
        }
        else {
            startFeeder();
        }
    }

    public void toggleReverseBoth() {
        if (m_state.is(ShooterStateEnum.REVERSE_BOTH)) {
            m_leftShooterPIDController.setReference(-500, ControlType.kVelocity);
            m_rightShooterPIDController.setReference(-500, ControlType.kVelocity);
            m_feederPIDController.setReference(-500, ControlType.kVelocity);
        }
        else {
            m_leftShooterPIDController.setReference(0, ControlType.kVelocity);
            m_rightShooterPIDController.setReference(0, ControlType.kVelocity);
            m_feederPIDController.setReference(0, ControlType.kVelocity);
        }
    }

    public void setBarExtension(double length) {
        // m_barPIDController.setReference(length, ControlType.kPosition);
    }

    public void toggleAutoAim() {
        if (m_state.is(ShooterStateEnum.AUTO_AIMING)) {
            stopAutoAim();
        }
        else {
            startAutoAim();
        }
    }

    public void startAutoAim() {
        m_state.addState(ShooterStateEnum.AUTO_AIMING);
    }

    public void stopAutoAim() {
        m_state.removeState(ShooterStateEnum.AUTO_AIMING);
    }

    public void increaseAngle() {
        setAngle(Units.radiansToDegrees(getAngle()) + 2);
    }

    public void decreaseAngle() {
        setAngle(Units.radiansToDegrees(getAngle()) - 2);
    }

    public double getAngle() {
        // return Math.PI - Math.acos((Math.pow((m_winchEncoder.getPosition() / 25) * (Constants.ShooterConstants.kPivotWinchAverageRadius * 2 * Math.PI) + 25, 2) - 462.25 - 52.128) / (-144.4)) + Units.degreesToRadians(16);
        // Desmos linear regression model
        return Units.degreesToRadians(57.8763 + (-1.07687 * m_winchEncoder.getPosition()));
    }

    private double calcAutoAim() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        Translation2d targetPose;


        if (alliance.isEmpty() || alliance.get() == Alliance.Red) {
            targetPose = new Translation2d(
                Units.inchesToMeters(652.73),
                Units.inchesToMeters(218.42)
            );
        }
        else {
            targetPose = new Translation2d(
                Units.inchesToMeters(-1.50),
                Units.inchesToMeters(218.42)
            );
        }

        double dist = m_poseSupplier.get().getTranslation().getDistance(targetPose);

        double height = Units.feetToMeters(6.6) + Units.inchesToMeters(5);
        double angle = Math.atan(height / dist);

        // double result = Units.radiansToDegrees(angle) + (2.3 * dist);
        // double result = Units.radiansToDegrees(angle) + (0.6 * dist * dist);
        double result = Units.radiansToDegrees(angle) + (0.5 * dist * dist);
        // System.out.println("Auto Aim Request: " + result + "Gravity Compensation: " + (0.5 * dist * dist));

        if (result > 59) {
            return 59;
        }
        else if (result < 32) {
            return 32;
        }
        else {
            return result;
        }
    }

    public Trigger getHasNoteTrigger() {
        return new Trigger(() -> m_state.is(ShooterStateEnum.HAS_NOTE));
    }

    public boolean atAngleSetpoint() {
        return Math.abs(m_desiredAngle - getAngle()) < 1;
    }

    public boolean getLimitSwitchState() {
        return m_angleLimitSwitch.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("pos", m_winchEncoder.getPosition());
        // SmartDashboard.putNumber("Len", (m_winchEncoder.getPosition() / 25) * (Constants.ShooterConstants.kPivotWinchAverageRadius * 2 * Math.PI) + 25);

        SmartDashboard.putBoolean("Limit Switch State", getLimitSwitchState());
        SmartDashboard.putBoolean("Shooter Spinning", m_state.is(ShooterStateEnum.SHOOTER_WHEEL_SPINNING));
        SmartDashboard.putBoolean("Feeder Spinning", m_state.is(ShooterStateEnum.FEEDER_WHEELS_SPINNING));

        if (m_state.is(ShooterStateEnum.AUTO_AIMING)) {
            setAngle(calcAutoAim());
        }

        m_currentAngleLog.log(getAngle());

        m_rpmLLog.log(-m_leftShooterEncoder.getVelocity());
        m_rpmRLog.log(m_rightShooterEncoder.getVelocity());

        m_hasNoteLog.log(m_state.is(ShooterStateEnum.HAS_NOTE));
        m_feederSensorLog.log(m_debouncedFeederSensorTrigger.getAsBoolean());
        
    }
}

