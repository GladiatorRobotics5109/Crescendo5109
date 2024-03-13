package frc.robot.subsystems.shooter;

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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.util.Constants.ShooterConstants;
import frc.robot.util.logging.Loggable;
import frc.robot.util.logging.LoggableBoolean;
import frc.robot.util.logging.LoggableDouble;
import frc.robot.stateMachine.ShooterState;
import frc.robot.stateMachine.StateMachine;
import frc.robot.stateMachine.ShooterState.ShooterStateEnum;
// import frc.robot.subsystems.logging.Loggable;
// import frc.robot.subsystems.logging.LoggableDouble;
// import frc.robot.subsystems.logging.Logger;
/**
 * Represents the Shooter
 */
public class ShooterSubsystem extends SubsystemBase {
    
    private final CANSparkMax m_leftShooterMotor;  // Left Side Shooter Wheels
    private final CANSparkMax m_rightShooterMotor; // Right Side Shooter Wheels
    private final CANSparkMax m_feederMotor; // Feeder Wheels
    private final CANSparkMax m_winchMotor; // Shooter Tilt Winch
    // private final CANSparkMax m_barMotor; // Amp Bar
    private final Servo m_barActuator;

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

    private double m_desiredAngle;


    private final Trigger m_feederSensorTrigger;
    private final Trigger m_debouncedFeederSensorTrigger;

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

    public ShooterSubsystem(Supplier<Pose2d> poseSupplier) {
        m_state = StateMachine.getShooterState();

        m_leftShooterMotor = new CANSparkMax(ShooterConstants.kLeftShooterMotorPort, MotorType.kBrushless);
        m_rightShooterMotor = new CANSparkMax(ShooterConstants.kRightShooterMotorPort, MotorType.kBrushless);
        m_feederMotor = new CANSparkMax(ShooterConstants.kFeederMotorPort, MotorType.kBrushless);
        m_winchMotor = new CANSparkMax(ShooterConstants.kWinchMotorPort, MotorType.kBrushless);
        // m_barMotor = new CANSparkMax(ShooterConstants.kBarMotorPort, MotorType.kBrushless);
        m_barActuator = new Servo(ShooterConstants.kBarActuatorChannel);


        m_leftShooterPIDController = m_leftShooterMotor.getPIDController();
        m_rightShooterPIDController = m_rightShooterMotor.getPIDController();

        m_feederPIDController = m_feederMotor.getPIDController();
        m_winchPIDController = m_winchMotor.getPIDController();
        // m_barPIDController = m_barMotor.getPIDController();

        m_leftShooterEncoder = m_leftShooterMotor.getEncoder();
        m_rightShooterEncoder = m_rightShooterMotor.getEncoder();
        m_feederEncoder = m_feederMotor.getEncoder();
        m_winchEncoder = m_winchMotor.getEncoder();
        // m_barEncoder = m_barMotor.getEncoder();

        m_leftShooterMotor.setIdleMode(IdleMode.kCoast);
        m_rightShooterMotor.setIdleMode(IdleMode.kCoast);
        m_feederMotor.setIdleMode(IdleMode.kBrake);
        m_winchMotor.setIdleMode(IdleMode.kBrake);
        // m_barMotor.setIdleMode(IdleMode.kBrake);

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
        m_feederSensorLog = new LoggableBoolean("HasNote", true);
        m_feederSensor2Log = new LoggableBoolean("Has Note2", true);

        // m_barPIDController.setP(ShooterConstants.kBarP);
        // m_barPIDController.setI(ShooterConstants.kBarI);
        // m_barPIDController.setD(ShooterConstants.kBarD);

        // m_shooterPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        // m_feederPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        // m_winchPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        // m_barPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);



        m_feederSensor = new DigitalInput(ShooterConstants.kFeederSensorChannel);

        //TODO: Need gear ratios and position conversion factor for bar and winch (maybe not winch)
        // m_barEncoder.setPositionConversionFactor(ShooterConstants.kBarPositionConversionFactor);

        m_winchEncoder.setPosition(0);


        m_feederSensorTrigger = new Trigger(() -> m_feederSensor.get());

        //negated since the beambreak will return true until beam is broken

        m_debouncedFeederSensorTrigger = m_feederSensorTrigger.debounce(0.01, DebounceType.kBoth).negate();
        // m_debouncedFeederSensorTrigger = m_feederSensorTrigger.negate();

        m_desiredAngle = 58;
        m_reachedAngleSetpointTrigger = new Trigger(() -> atAngleSetpoint());


        configureBindings();
    }
    /**
     * These bindings make it so that when a note is in the shooter, the shooter wheels are spinning and ready to shoot
     * 
     */
    private void configureBindings() {
        m_debouncedFeederSensorTrigger.onTrue(
            Commands.sequence(
                getAddHasNoteStateCommand(),
                getStopShooterCommand(),
                Commands.runOnce(() -> {
                    m_feederMotor.set(0.1);
                    m_leftShooterMotor.set(0.001);
                    m_rightShooterMotor.set(-0.001);
                }),
                Commands.waitSeconds(0.15),
                getStopFeederCommand(),
                getStopShooterCommand()
            )
         );

         m_debouncedFeederSensorTrigger.whileFalse(Commands.run(() -> m_state.removeState(ShooterStateEnum.HAS_NOTE)));
    }

    public Command getAimAmpCommand() {
        return this.runOnce(() -> {
            setAngle(58);
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

    public Command getExtendBarCommand() {
        return this.runOnce(() -> {
            m_barActuator.set(1);
        });
    }

    public Command getRetractBarCommand() {
        return this.runOnce(() -> {
            m_barActuator.set(0);
        });
    }

    public Command getToggleBarCommand() {
        
        return this.runOnce(() -> {
            if (m_state.is(ShooterStateEnum.BAR_EXTENDED)) {
                resetBar();
            } else {
                extendBar();
            }
        });
    }

    public void extendBar() {
        m_barActuator.set(1.0);
        m_state.addState(ShooterStateEnum.BAR_EXTENDED);
    }

    public void resetBar() {
        m_barActuator.set(0);
        m_state.removeState(ShooterStateEnum.BAR_EXTENDED);
    }

    public void setAngle(double angle) {
        if (angle > 58) {
            System.out.println("MAX: " + angle);
            angle = 58;
        }
        if (angle < 38) {
            System.out.println("MIN: " + angle);
            angle = 38;
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
        m_leftShooterMotor.set(-0.24);
        m_rightShooterMotor.set(0.24);
        extendBar();
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

        return Units.radiansToDegrees(angle) + (2.5 * dist);
    }

    public Trigger getHasNoteTrigger() {
        return new Trigger(() -> m_state.is(ShooterStateEnum.HAS_NOTE));
    }

    public boolean atAngleSetpoint() {
        return Math.abs(m_desiredAngle - getAngle()) < 1;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("pos", m_winchEncoder.getPosition());
        // SmartDashboard.putNumber("Len", (m_winchEncoder.getPosition() / 25) * (Constants.ShooterConstants.kPivotWinchAverageRadius * 2 * Math.PI) + 25);

        if (m_state.is(ShooterStateEnum.AUTO_AIMING)) {
            setAngle(calcAutoAim());
        }

        m_currentAngleLog.log(getAngle());

        m_rpmLLog.log(-m_leftShooterEncoder.getVelocity());
        m_rpmRLog.log(m_rightShooterEncoder.getVelocity());

        m_feederSensorLog.log(m_state.is(ShooterStateEnum.HAS_NOTE));
        m_feederSensor2Log.log(m_feederSensorTrigger .getAsBoolean());

        // m_state.removeState(ShooterStateEnum.HAS_NOTE);
    }
}

