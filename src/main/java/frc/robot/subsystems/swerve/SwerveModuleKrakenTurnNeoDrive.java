package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Conversions;
import frc.robot.RevOptimizer;

/**
 * Represents a swerve module with a Kraken (TalonFX) turn motor and a NEO (SparkMAX) drive motor.
 */
public class SwerveModuleKrakenTurnNeoDrive extends SwerveModule {
    private final Translation2d m_modulePos;
    private final String m_moduleName;
    private final int m_moduleNum;

    private final CANSparkMax m_driveMotor;
    private final TalonFX m_turnMotor;

    private final RelativeEncoder m_driveEncoder;

    private final SparkPIDController m_drivePIDController;


    public SwerveModuleKrakenTurnNeoDrive(Translation2d modulePos, String moduleName, int moduleNum, int driveMotorPort, int turnMotorPort) {
        m_modulePos = modulePos;
        m_moduleName = moduleName;
        m_moduleNum = moduleNum;

        m_driveMotor = new CANSparkMax(driveMotorPort, MotorType.kBrushless);
        m_turnMotor = new TalonFX(turnMotorPort);

        m_driveMotor.setIdleMode(IdleMode.kCoast);
        m_turnMotor.setNeutralMode(NeutralModeValue.Coast);

        m_driveEncoder = m_driveMotor.getEncoder();

        m_drivePIDController = m_driveMotor.getPIDController();

        // set drive PID values
        m_drivePIDController.setP(0.3);
        m_drivePIDController.setI(0.0);
        m_drivePIDController.setD(0.0);

        // TODO: turn motor PID tune
        TalonFXConfiguration turnMotorConfiguration = new TalonFXConfiguration();
        turnMotorConfiguration.Slot0.kP = 1.5;
        turnMotorConfiguration.Slot0.kI = 0.0;
        turnMotorConfiguration.Slot0.kI = 0.0;
        turnMotorConfiguration.Feedback.RotorToSensorRatio = Constants.SwerveConstants.kSwerveTurnGearRatio;

        m_turnMotor.getConfigurator().apply(turnMotorConfiguration);
        
        m_driveEncoder.setVelocityConversionFactor(1 / (Constants.SwerveConstants.kNeoTicksPerWheelRadian) * Constants.SwerveConstants.kWheelRadius);
        m_driveEncoder.setPositionConversionFactor(Constants.SwerveConstants.kNeoTicksPerRevolution / (Constants.SwerveConstants.kNeoTicksPerWheelRadian) * Constants.SwerveConstants.kWheelRadius);
        // unlike the SparkMAXes, we have to run kraken ticks -> radian conversion manually in our code.
        // m_turnMotor.getSelectedSensorPosition() should never really be used in this class to get encoder position
        // getTurnEncoderPositionRad() should be used instead.
    }

    @Override
    public Translation2d getPos() {
        return m_modulePos;
    }

    @Override
    public void setDesiredState(SwerveModuleState state, boolean optimize) {
        SwerveModuleState optimizedState = optimize ? RevOptimizer.optimize(state, Rotation2d.fromRadians(getTurnWheelPositionRad())) : state;

        m_drivePIDController.setReference(optimizedState.speedMetersPerSecond, ControlType.kVelocity);
        m_turnMotor.setPosition(optimizedState.angle.getRotations() * (1 / Constants.SwerveConstants.kSwerveTurnGearRatio));
    }

    @Override
    public void setDesiredState(SwerveModuleState state) {
        setDesiredState(state, true);
    }

    @Override
    public void brakeAll() {
        m_driveMotor.setIdleMode(IdleMode.kBrake);
        m_turnMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void coastAll() {
        m_driveMotor.setIdleMode(IdleMode.kCoast);
        m_turnMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public String getName() {
        return m_moduleName;
    }

    @Override
    public int getNumber() {
        return m_moduleNum;
    }

    @Override
    public void resetEncoders() {
        m_driveEncoder.setPosition(0);
    }

    private double getTurnWheelPositionRad() {
        return Conversions.krakenToRad(m_turnMotor.getPosition().getValueAsDouble(), Constants.SwerveConstants.kKrakenTicksPerTurnWheelRadian);
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveEncoder.getVelocity(), Rotation2d.fromRadians(m_turnMotor.getPosition().getValueAsDouble()));
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(m_driveEncoder.getPosition(), Rotation2d.fromRadians(m_turnMotor.getPosition().getValueAsDouble()));
    }
}
