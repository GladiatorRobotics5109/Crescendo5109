package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;


import java.util.Optional;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;

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

    private final BangBangController m_leftShooterBangController;
    private final BangBangController m_rightShooterBangController;

    private final SimpleMotorFeedforward m_leftShooterFeedForward;
    private final SimpleMotorFeedforward m_rightShooterFeedForward;

    private final RelativeEncoder m_leftShooterEncoder;
    private final RelativeEncoder m_rightShooterEncoder;
    private final RelativeEncoder m_feederEncoder;
    private final RelativeEncoder m_winchEncoder;

    private final DigitalInput m_feederSensor;
    private final DigitalInput m_intakeSensor;
    private final DigitalInput m_angleLimitSwitch;

    private double m_desiredAngle;


    private final Trigger m_feederSensorTrigger;
    private final Trigger m_intakeSensorTrigger;
    private final Trigger m_combinedSensorTrigger;


    private final Trigger m_limitSwitchTrigger;

    private final Trigger m_reachedAngleSetpointTrigger;

    private Supplier<Pose2d> m_poseSupplier;

    private final ShooterState m_state;

    // private final LoggableDouble m_desiredRps;
    // private final LoggableDouble m_currentRps;

    private final LoggableDouble m_desiredAngleLog;
    private final LoggableDouble m_currentAngleLog;
    private final LoggableBoolean m_autoAimingLog;

    private final LoggableBoolean m_feederSensorLog;
    private final LoggableBoolean m_feederSensor2Log;

    private final LoggableDouble m_rpmLLog;
    private final LoggableDouble m_rpmRLog;

    private final LoggableDouble m_desiredVoltageLeftLog;
    private final LoggableDouble m_desiredVoltageRightLog;

    private final MutableMeasure<Voltage> m_appliedVoltage = MutableMeasure.mutable(Volts.of(0));
    private final MutableMeasure<Angle> m_angle = MutableMeasure.mutable(Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> m_velocity = MutableMeasure.mutable(RotationsPerSecond.of(0));
    private final SysIdRoutine m_routine;

    private double m_leftShooterDesiredRPM = 0;
    private double m_rightShooterDesiredRPM = 0;

    public ShooterSubsystem(Supplier<Pose2d> poseSupplier) {
        m_state = StateMachine.getShooterState();

        m_leftShooterMotor = new CANSparkMax(ShooterConstants.kLeftShooterMotorPort, MotorType.kBrushless);
        m_rightShooterMotor = new CANSparkMax(ShooterConstants.kRightShooterMotorPort, MotorType.kBrushless);
        m_feederMotor = new CANSparkMax(ShooterConstants.kFeederMotorPort, MotorType.kBrushless);
        m_winchMotor = new CANSparkMax(ShooterConstants.kWinchMotorPort, MotorType.kBrushless);
        m_leftBarServo = new Servo(ShooterConstants.kLeftBarServoChannel);
        m_rightBarServo = new Servo(ShooterConstants.kRightBarServoChannel);

        m_leftShooterMotor.setInverted(true);
        //working frequence: 50 - 333 Hz
        //dead band: 3 us/ 2us ?????
        //PWM Range: 500 us to 2500 us (270 degrees)
        m_leftBarServo.setBoundsMicroseconds(2500, 1504, 1500, 1496, 500);
        m_rightBarServo.setBoundsMicroseconds(500, 1501, 1500, 1499, 2500);

        m_leftShooterPIDController = m_leftShooterMotor.getPIDController();
        m_rightShooterPIDController = m_rightShooterMotor.getPIDController();

        //Controller in RPM
        m_leftShooterBangController = new BangBangController(50.0); 
        m_rightShooterBangController = new BangBangController(50.0);

        m_leftShooterFeedForward = new SimpleMotorFeedforward(0.12845, 0.0020526); //ks: 0.12845
        m_rightShooterFeedForward = new SimpleMotorFeedforward(0.24763, 0.0020564); //ks: 0.

        m_feederPIDController = m_feederMotor.getPIDController();
        m_winchPIDController = m_winchMotor.getPIDController();

        m_leftShooterEncoder = m_leftShooterMotor.getEncoder();
        m_rightShooterEncoder = m_rightShooterMotor.getEncoder();
        m_feederEncoder = m_feederMotor.getEncoder();
        m_winchEncoder = m_winchMotor.getEncoder();

        m_leftShooterMotor.setIdleMode(IdleMode.kCoast);
        m_rightShooterMotor.setIdleMode(IdleMode.kCoast);
        m_feederMotor.setIdleMode(IdleMode.kBrake);
        m_winchMotor.setIdleMode(IdleMode.kBrake);

        m_leftShooterPIDController.setP(ShooterConstants.kShooterP);
        m_leftShooterPIDController.setI(ShooterConstants.kShooterI);
        m_leftShooterPIDController.setD(ShooterConstants.kShooterD);

        m_rightShooterPIDController.setP(ShooterConstants.kShooterP);
        m_rightShooterPIDController.setI(ShooterConstants.kShooterI);
        m_rightShooterPIDController.setD(ShooterConstants.kShooterD);

        m_feederEncoder.setPositionConversionFactor(ShooterConstants.kFeederPositionConversionFactor);

        m_feederPIDController.setP(ShooterConstants.kFeederP);
        m_feederPIDController.setI(ShooterConstants.kFeederI);
        m_feederPIDController.setIZone(5);
        m_feederPIDController.setD(ShooterConstants.kFeederD);

        m_winchPIDController.setP(ShooterConstants.kWinchP);
        m_winchPIDController.setI(ShooterConstants.kWinchI);
        m_winchPIDController.setD(ShooterConstants.kWinchD);
        m_winchPIDController.setFF(0.001);

        // m_winchEncoder.setPositionConversionFactor(ShooterConstants.kWinchPositionConversionFactor);
        // m_winchEncoder.setPosition();

        m_poseSupplier = poseSupplier;

        m_desiredAngleLog = new LoggableDouble("Shooter Desired Angle", true);
        m_currentAngleLog = new LoggableDouble("Shooter Current Angle", true);
        m_autoAimingLog = new LoggableBoolean("Shooter Auto Aiming", true);
        m_rpmLLog = new LoggableDouble("Shooter L RPM", true);
        m_rpmRLog = new LoggableDouble("Shooter R RPM", true);
        m_feederSensorLog = new LoggableBoolean("HasNote", true);
        m_feederSensor2Log = new LoggableBoolean("Has Note2", true);

        m_desiredVoltageLeftLog = new LoggableDouble("Left Shooter Desired Voltage", true);
        m_desiredVoltageRightLog = new LoggableDouble("Right Shooter Desired Voltage", true);

        m_overrideMinMaxAngle = false;


        m_feederSensor = new DigitalInput(ShooterConstants.kFeederSensorChannel);
        m_intakeSensor = new DigitalInput(ShooterConstants.kIntakeSensorChannel);
        m_angleLimitSwitch = new DigitalInput(ShooterConstants.kLimitSwitchChannel);

        m_winchEncoder.setPosition((50 - 57.8763) / (-1.07687));


        //negated since the beambreak will return true until beam is broken
        m_feederSensorTrigger = new Trigger(() -> m_feederSensor.get()).debounce(0.1, DebounceType.kBoth).negate();
        m_intakeSensorTrigger = new Trigger(() -> m_intakeSensor.get()).debounce(0.1, DebounceType.kBoth).negate();
        m_combinedSensorTrigger = m_feederSensorTrigger.and(() -> m_intakeSensor.get());
        m_limitSwitchTrigger = new Trigger(() -> m_angleLimitSwitch.get()).negate();




        m_desiredAngle = 52;
        m_reachedAngleSetpointTrigger = new Trigger(() -> atAngleSetpoint());

        m_routine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(this::voltageDrive, this::logMotors, this)
        );

        configureBindings();
    }


    private void voltageDrive(Measure<Voltage> volts) {
        m_leftShooterMotor.set(volts.in(Volts) / RobotController.getBatteryVoltage());
        m_rightShooterMotor.set(volts.in(Volts) / RobotController.getBatteryVoltage());
    }

    private void logMotors(SysIdRoutineLog log) {
        log.motor("shooter-left")
        .voltage(
            m_appliedVoltage.mut_replace(
                m_leftShooterMotor.get() * RobotController.getBatteryVoltage(), Volts))
        .angularPosition(m_angle.mut_replace(m_leftShooterEncoder.getPosition(), Rotations))
        .angularVelocity(m_velocity.mut_replace(m_leftShooterEncoder.getVelocity(), RotationsPerSecond));
        
        log.motor("shooter-right")
        .voltage(
            m_appliedVoltage.mut_replace(
                m_rightShooterMotor.get() * RobotController.getBatteryVoltage(), Volts))
        .angularPosition(m_angle.mut_replace(m_rightShooterEncoder.getPosition(), Rotations))
        .angularVelocity(m_velocity.mut_replace(m_rightShooterEncoder.getVelocity(), RotationsPerSecond));
    }

    public Command getSysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
        return m_routine.quasistatic(direction);
    }

    public Command getSysIdDynamicCommand(SysIdRoutine.Direction direction) {
        return m_routine.dynamic(direction);
    }

    /**
     * These bindings make it so that when a note is in the shooter, the shooter wheels are spinning and ready to shoot
     * 
     */
    private void configureBindings() {
        m_feederSensorTrigger.onTrue(
            Commands.sequence(
                getAddHasNoteStateCommand(),
                //getStopShooterCommand(),
                Commands.runOnce(() -> {
                    m_feederPIDController.setReference(m_feederEncoder.getPosition() + 1, ControlType.kPosition);
                    //m_feederMotor.set(0.1);
                    // m_leftShooterMotor.set(0.001);
                    // m_rightShooterMotor.set(-0.001);
                })//,
                // Commands.waitSeconds(0.15),
                //getStopFeederCommand(),
                //getStopShooterCommand()
            )
         );

        // m_feederSensorTrigger.whileFalse(Commands.runOnce(() -> {
        //     m_state.removeState(ShooterStateEnum.HAS_NOTE);
        //  }));

        m_feederSensorTrigger.onFalse(
            Commands.sequence(
                Commands.waitSeconds(0.5),
                getStopShooterCommand(),
                getStopFeederCommand()
            ));

        m_intakeSensorTrigger.onTrue(
            Commands.run(() -> {
                    m_feederMotor.set(0.4);
                }
            ));

        m_combinedSensorTrigger.onTrue(Commands.run(() -> {
            m_state.addState(ShooterStateEnum.HAS_NOTE);
         }));

        //If the trigger is turned false while the intake sensor is activated, the note is slipping out.
        m_combinedSensorTrigger.onFalse(Commands.run(() -> {
            if (m_intakeSensor.get()) {
                m_feederPIDController.setReference(m_feederEncoder.getPosition() - 0.5, ControlType.kPosition);
            } else {
                m_state.removeState(ShooterStateEnum.HAS_NOTE);
            }
        }));

        m_limitSwitchTrigger.onTrue(Commands.run(() -> {
        m_winchEncoder.setPosition((50 - 57.8763) / (-1.07687));
        setAngle(50);
        }));
    }

    public Command getAimAmpCommand() {
        return this.runOnce(() -> {
            setAngle(52);
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
            if (m_state.is(ShooterStateEnum.BAR_SWUNG)) {
                resetBar();
            } else {
                swingBar();
            }
        });
    }



    public void swingBar() {
        //m_leftBarServo.setPulseTimeMicroseconds(2000);
        m_leftBarServo.setSpeed(1.0);
        //m_rightBarServo.setPosition(1);
        m_state.addState(ShooterStateEnum.BAR_SWUNG);
    }

    public void resetBar() {
        m_leftBarServo.setPosition(0);
        //m_rightBarServo.setPosition(0);
        m_state.removeState(ShooterStateEnum.BAR_SWUNG);
    }

    public void setLimitSwitchAngle() {
        m_winchEncoder.setPosition((50 - 57.8763) / (-1.07687));
    }


    public void setAngle(double angle) {
        if (!m_overrideMinMaxAngle && angle > 52) {
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

    

    /**
     * Sets the angle of the shooter based on the law of cosines and two static angles.
     * 
     * @param angle desired shooter angle in degrees 
     */
    // public void setAngle(double angle) {
    //     if (angle > 57) {
    //         System.out.println("MAX: " + angle);
    //     } else if (angle < 30) {
    //         System.out.println("Min: " + angle);
    //     }

    //     double a = Math.sqrt(Math.pow(10.37, 2) + Math.pow(1.77, 2));
    //     double b = Math.sqrt(Math.pow(7, 2) + Math.pow(1.75, 2));
    //     double theta_1 = Units.degreesToRadians(14.04);
    //     double theta_2 = Units.degreesToRadians(25.41);

    //     double desiredD = Math.PI - theta_1 - theta_2 - angle;

    //     double desiredLength = Math.sqrt(Math.pow(a,2) + Math.pow(b,2) - 2 * a * b * Math.cos(Units.degreesToRadians(angle)));

    //     m_winchPIDController.setReference(desiredLength, ControlType.kPosition);
    // }   

    //TODO: Figure out optimal RPM for shooter
    public void startShooter() {

        m_leftShooterDesiredRPM = 5500;
        m_rightShooterDesiredRPM = 5500;
        m_state.addState(ShooterStateEnum.SHOOTER_WHEEL_SPINNING);
    }

    public void stopShooter() {
        //m_leftShooterPIDController.setReference(0, ControlType.kVelocity);
        //m_rightShooterPIDController.setReference(0, ControlType.kVelocity);
        // m_leftShooterMotor.set(0);
        // m_rightShooterMotor.set(0);
        m_leftShooterDesiredRPM = 0;
        m_rightShooterDesiredRPM = 0;
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
        m_leftShooterMotor.set(-0.24);
        m_rightShooterMotor.set(0.24);
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
        double result = Units.radiansToDegrees(angle) + (0.6 * dist * dist);
        System.out.print("Auto Aim Request: " + result + "Gravity Compensation: " + (0.6 * dist * dist));

        if (result > 52) {
            return 52;
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

        // Setting shooter voltage
        if (m_leftShooterDesiredRPM == 0) {
            m_leftShooterMotor.set(0);
        } else {

            double leftVoltage = m_leftShooterBangController.calculate(m_leftShooterEncoder.getVelocity(), 5500) 
            + m_leftShooterFeedForward.calculate(5500);

            m_leftShooterMotor.setVoltage(leftVoltage);
            m_desiredVoltageLeftLog.log(leftVoltage);
        }

        if (m_rightShooterDesiredRPM == 0) {
            m_rightShooterMotor.set(0);
        } else {
            double rightVoltage = m_rightShooterBangController.calculate(m_rightShooterEncoder.getVelocity(), 5500) 
            +  m_rightShooterFeedForward.calculate(5500);

            m_rightShooterMotor.setVoltage(rightVoltage);
            m_desiredVoltageRightLog.log(rightVoltage);
        }
        
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

        m_feederSensorLog.log(m_state.is(ShooterStateEnum.HAS_NOTE));
        m_feederSensor2Log.log(m_feederSensorTrigger.getAsBoolean());
    }
}

