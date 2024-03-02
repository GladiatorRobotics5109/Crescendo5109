package frc.robot.subsystems.shooter;

import java.util.Optional;
import java.util.function.Supplier;

import javax.xml.namespace.QName;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.AccelStrategy;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.util.Constants.ShooterConstants;
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

    // private final DigitalInput m_feederSensor;


    // private final Trigger m_feederSensorTrigger;
    // private final Trigger m_debouncedFeederSensorTrigger;

    private Supplier<Pose2d> m_poseSupplier;

    private final ShooterState m_state;

    // private final LoggableDouble m_desiredRps;
    // private final LoggableDouble m_currentRps;

    private final LoggableDouble m_desiredAngle;
    private final LoggableDouble m_currentAngle;
    private final LoggableBoolean m_autoAiming;


    public ShooterSubsystem(Supplier<Pose2d> poseSupplier) {

        m_state = StateMachine.getShooterState();

        m_leftShooterMotor = new CANSparkMax(ShooterConstants.kLeftShooterMotorPort, MotorType.kBrushless);
        m_rightShooterMotor = new CANSparkMax(ShooterConstants.kRightShooterMotorPort, MotorType.kBrushless);
        m_feederMotor = new CANSparkMax(ShooterConstants.kFeederMotorPort, MotorType.kBrushless);
        m_winchMotor = new CANSparkMax(ShooterConstants.kWinchMotorPort, MotorType.kBrushless);
        // m_barMotor = new CANSparkMax(ShooterConstants.kBarMotorPort, MotorType.kBrushless);

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
        m_feederMotor.setIdleMode(IdleMode.kCoast);
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

        m_desiredAngle = new LoggableDouble("Shooter Desired Angle", true);
        m_currentAngle = new LoggableDouble("Shooter Current Angle", true);
        m_autoAiming = new LoggableBoolean("Shooter Auto Aiming", true);

        // m_barPIDController.setP(ShooterConstants.kBarP);
        // m_barPIDController.setI(ShooterConstants.kBarI);
        // m_barPIDController.setD(ShooterConstants.kBarD);

        // m_shooterPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        // m_feederPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        // m_winchPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        // m_barPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);


        // m_feederSensor = new DigitalInput(ShooterConstants.kFeederSensorChannel);

        //TODO: Need gear ratios and position conversion factor for bar and winch (maybe not winch)
        // m_barEncoder.setPositionConversionFactor(ShooterConstants.kBarPositionConversionFactor);

        m_winchEncoder.setPosition(0);


        // m_feederSensorTrigger = new Trigger(() -> m_feederSensor.get());

        //negated since the beambreak will return true until beam is broken
 
        // m_debouncedFeederSensorTrigger = m_feederSensorTrigger.debounce(0.1).negate();

        configureBindings();
    }
    /**
     * These bindings make it so that when a note is in the shooter, the shooter wheels are spinning and ready to shoot
     * 
     */
    private void configureBindings() {
        
        // m_debouncedFeederSensorTrigger.onTrue(
        //     Commands.sequence(
        //         getRemoveHasNoteStateCommand(),
        //         getStopFeederCommand(),
        //         getStopShooterCommand()
        //     )
        // );
    }


    public Command getAimAmpCommand() {
        return this.runOnce(() -> {
            // setAngle(Units.degreesToRadians(58));
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

    public void setAngle(double angle) {
        if (!(angle < 58 && angle > 38)) {
            System.out.println(angle);
            // System.out.println("NONONO");

            return;
        }        
        double desiredRot = (angle - 57.8763) / (-1.07687);

        m_winchPIDController.setReference(desiredRot, ControlType.kPosition);
    }

    //TODO: Figure out optimal RPM for shooter
    public void startShooter() {
        // m_desiredRps.log(2000.0);
        //m_leftShooterPIDController.setReference(5000, ControlType.kVelocity);
        //m_rightShooterPIDController.setReference(5000, ControlType.kVelocity);
        // m_leftShooterMotor.set(-1);
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

        m_desiredAngle.log(angle);
        return Units.radiansToDegrees(angle);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("pos", m_winchEncoder.getPosition());
        // SmartDashboard.putNumber("Len", (m_winchEncoder.getPosition() / 25) * (Constants.ShooterConstants.kPivotWinchAverageRadius * 2 * Math.PI) + 25);

        if (m_state.is(ShooterStateEnum.AUTO_AIMING)) {
            setAngle(calcAutoAim());
        }

        m_currentAngle.log(getAngle());
        m_autoAiming.log(m_state.is(ShooterStateEnum.AUTO_AIMING));
    }
}

