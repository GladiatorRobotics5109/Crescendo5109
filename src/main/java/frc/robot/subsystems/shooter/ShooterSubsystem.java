package frc.robot.subsystems.shooter;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.AccelStrategy;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.ShooterConstants;
import frc.robot.stateMachine.ShooterState;
import frc.robot.stateMachine.StateMachine;
import frc.robot.stateMachine.ShooterState.ShooterStateEnum;
/**
 * Represents the Shooter
 */
public class ShooterSubsystem extends SubsystemBase {
    
    private final CANSparkMax m_leftShooterMotor;  // Left Side Shooter Wheels
    private final CANSparkMax m_rightShooterMotor; // Right Side Shooter Wheels
    private final CANSparkMax m_feederMotor; // Feeder Wheels
    private final CANSparkMax m_winchMotor; // Shooter Tilt Winch
    private final CANSparkMax m_barMotor; // Amp Bar

    private final SparkPIDController m_leftShooterPIDController;
    private final SparkPIDController m_rightShooterPIDController;
    private final SparkPIDController m_feederPIDController;
    private final SparkPIDController m_winchPIDController;
    private final SparkPIDController m_barPIDController;

    private final RelativeEncoder m_leftShooterEncoder;
    private final RelativeEncoder m_rightShooterEncoder;
    private final RelativeEncoder m_feederEncoder;
    private final AbsoluteEncoder m_tiltEncoder;
    private final RelativeEncoder m_barEncoder;

    private final DigitalInput m_feederSensor;


    private final Trigger m_feederSensorTrigger;
    private final Trigger m_debouncedFeederSensorTrigger;

    private final ShooterState m_state;


    public ShooterSubsystem() {

        m_state = StateMachine.getShooterState();

        m_leftShooterMotor = new CANSparkMax(ShooterConstants.kLeftShooterMotorPort, MotorType.kBrushless);
        m_rightShooterMotor = new CANSparkMax(ShooterConstants.kRightShooterMotorPort, MotorType.kBrushless);
        m_feederMotor = new CANSparkMax(ShooterConstants.kFeederMotorPort, MotorType.kBrushless);
        m_winchMotor = new CANSparkMax(ShooterConstants.kWinchMotorPort, MotorType.kBrushless);
        m_barMotor = new CANSparkMax(ShooterConstants.kBarMotorPort, MotorType.kBrushless);

        m_leftShooterPIDController = m_leftShooterMotor.getPIDController();
        m_rightShooterPIDController = m_rightShooterMotor.getPIDController();
    
        m_feederPIDController = m_feederMotor.getPIDController();
        m_winchPIDController = m_winchMotor.getPIDController();
        m_barPIDController = m_barMotor.getPIDController();

        m_leftShooterEncoder = m_leftShooterMotor.getEncoder();
        m_rightShooterEncoder = m_rightShooterMotor.getEncoder();
        m_feederEncoder = m_feederMotor.getEncoder();
        m_tiltEncoder = m_winchMotor.getAbsoluteEncoder(Type.kDutyCycle);
        m_barEncoder = m_barMotor.getEncoder();

        m_leftShooterMotor.setIdleMode(IdleMode.kCoast);
        m_rightShooterMotor.setIdleMode(IdleMode.kCoast);
        m_feederMotor.setIdleMode(IdleMode.kCoast);
        m_winchMotor.setIdleMode(IdleMode.kBrake);
        m_barMotor.setIdleMode(IdleMode.kBrake);

        m_leftShooterPIDController.setP(ShooterConstants.kShooterP);
        m_leftShooterPIDController.setI(ShooterConstants.kShooterI);
        m_leftShooterPIDController.setD(ShooterConstants.kShooterD);

        m_rightShooterPIDController.setP(ShooterConstants.kShooterP);
        m_rightShooterPIDController.setI(ShooterConstants.kShooterI);
        m_rightShooterPIDController.setD(ShooterConstants.kShooterD);

        m_feederPIDController.setP(ShooterConstants.kFeederP);
        m_feederPIDController.setI(ShooterConstants.kFeederI);
        m_feederPIDController.setD(ShooterConstants.kFeederD);

        m_winchPIDController.setP(ShooterConstants.kWinchP);
        m_winchPIDController.setI(ShooterConstants.kWinchI);
        m_winchPIDController.setD(ShooterConstants.kWinchD);

        m_barPIDController.setP(ShooterConstants.kBarP);
        m_barPIDController.setI(ShooterConstants.kBarI);
        m_barPIDController.setD(ShooterConstants.kBarD);

        // m_shooterPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        // m_feederPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        m_winchPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        m_barPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);


        m_feederSensor = new DigitalInput(ShooterConstants.kFeederSensorChannel);

        //TODO: Need gear ratios and position conversion factor for bar and winch (maybe not winch)
        m_barEncoder.setPositionConversionFactor(ShooterConstants.kBarPositionConversionFactor);


        m_feederSensorTrigger = new Trigger(() -> m_feederSensor.get());

        //negated since the beambreak will return true until beam is broken
 
        m_debouncedFeederSensorTrigger = m_feederSensorTrigger.debounce(0.1).negate();

        configureBindings();
    }
    /**
     * These bindings make it so that when a note is in the shooter, the shooter wheels are spinning and ready to shoot
     * 
     */
    private void configureBindings() {
        
        m_debouncedFeederSensorTrigger.onTrue(
            Commands.sequence(
                getRemoveHasNoteStateCommand(),
                getStopFeederCommand(),
                getStopShooterCommand()
            )
        );
    }


    public Command getAimAmpCommand() {
        return this.runOnce(() -> {
            setAngle(Units.degreesToRadians(58));
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

    public void setAngle(double angle) {
        m_winchPIDController.setReference(angle, ControlType.kPosition);
    }

    //TODO: Figure out optimal RPM for shooter
    public void startShooter() {
        m_leftShooterPIDController.setReference(2000, ControlType.kVelocity);
        m_rightShooterPIDController.setReference(2000, ControlType.kVelocity);
        m_state.addState(ShooterStateEnum.SHOOTER_WHEEL_SPINNING);
    }

    public void stopShooter() {
        m_leftShooterPIDController.setReference(0, ControlType.kVelocity);
        m_rightShooterPIDController.setReference(0, ControlType.kVelocity);
        m_state.removeState(ShooterStateEnum.SHOOTER_WHEEL_SPINNING);
    }

    //TODO: Figure out optimal RPM for feeder
    public void startFeeder() {
        m_feederPIDController.setReference(500, ControlType.kVelocity);
        m_state.addState(ShooterStateEnum.FEEDER_WHEELS_SPINNING);
    }

    public void stopFeeder() {
        m_feederPIDController.setReference(0, ControlType.kVelocity);
        m_state.removeState(ShooterStateEnum.FEEDER_WHEELS_SPINNING);
    }

    public void setBarExtension(double length) {
        m_barPIDController.setReference(length, ControlType.kPosition);
    }



}

