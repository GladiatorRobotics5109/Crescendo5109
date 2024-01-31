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
public class SwerveModuleKrakenTurnNeoDrive {
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
        m_drivePIDController.setP(Constants.ModuleConstants.kDriveP);
        m_drivePIDController.setI(Constants.ModuleConstants.kDriveI);
        m_drivePIDController.setD(Constants.ModuleConstants.kDriveD);

        // TODO: turn motor PID tune
        TalonFXConfiguration turnMotorConfiguration = new TalonFXConfiguration();
        turnMotorConfiguration.Slot0.kP = Constants.ModuleConstants.kTurnP;
        turnMotorConfiguration.Slot0.kI = Constants.ModuleConstants.kTurnI;
        turnMotorConfiguration.Slot0.kD = Constants.ModuleConstants.kTurnD;
        turnMotorConfiguration.Feedback.RotorToSensorRatio = Constants.ModuleConstants.kSwerveTurnGearRatio;

        m_turnMotor.getConfigurator().apply(turnMotorConfiguration);
        
        m_driveEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kDrivePositionConversionFactor);
        m_driveEncoder.setPositionConversionFactor(Constants.ModuleConstants.kDriveVelocityConversionFactor);
        // unlike the SparkMAXes, we have to run kraken ticks -> radian conversion manually in our code.
        // m_turnMotor.getSelectedSensorPosition() should never really be used in this class to get encoder position
        // getTurnEncoderPositionRad() should be used instead.
    }

    
    public Translation2d getPos() {
        return m_modulePos;
    }

    
    public void setDesiredState(SwerveModuleState state, boolean optimize) {
        SwerveModuleState optimizedState = optimize ? RevOptimizer.optimize(state, Rotation2d.fromRadians(getTurnWheelPositionRad())) : state;

        m_drivePIDController.setReference(optimizedState.speedMetersPerSecond, ControlType.kVelocity);
        m_turnMotor.setPosition(optimizedState.angle.getRotations() * (1 / Constants.ModuleConstants.kSwerveTurnGearRatio));
    }

    
    public void setDesiredState(SwerveModuleState state) {
        setDesiredState(state, true);
    }

    
    public void brakeAll() {
        m_driveMotor.setIdleMode(IdleMode.kBrake);
        m_turnMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    
    public void coastAll() {
        m_driveMotor.setIdleMode(IdleMode.kCoast);
        m_turnMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    
    public String getName() {
        return m_moduleName;
    }

    
    public int getNumber() {
        return m_moduleNum;
    }

    
    public void resetEncoders() {
        m_driveEncoder.setPosition(0);
    }

    private double getTurnWheelPositionRad() {
        return Conversions.krakenToRad(m_turnMotor.getPosition().getValueAsDouble(), Constants.ModuleConstants.kKrakenTicksPerTurnWheelRadian);
    }

    
    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveEncoder.getVelocity(), Rotation2d.fromRadians(m_turnMotor.getPosition().getValueAsDouble()));
    }

    
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(m_driveEncoder.getPosition(), Rotation2d.fromRadians(m_turnMotor.getPosition().getValueAsDouble()));
    }
}
