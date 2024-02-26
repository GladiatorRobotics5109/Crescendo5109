package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Conversions;
import frc.robot.RevOptimizer;
import frc.robot.subsystems.logging.LoggableDouble;
import frc.robot.subsystems.logging.Logger;

/**
 * Represents a swerve module with a Kraken (TalonFX) turn motor and a NEO (SparkMAX) drive motor.
 */
public class SwerveModuleNeoTurnKrakenDrive {
    private final Translation2d m_modulePos;
    private final String m_moduleName;
    private final int m_moduleNum;

    private final CANSparkMax m_turnMotor;
    private final TalonFX m_driveMotor;

    //private final RelativeEncoder m_turnEncoder;
    private final AbsoluteEncoder m_turnAbsEncoder;

    private final SparkPIDController m_turnPIDController;

    private final LoggableDouble m_rpsLog;
    private final LoggableDouble m_desiredSpeedLog;
    private final LoggableDouble m_currentSpeedLog;


    public SwerveModuleNeoTurnKrakenDrive(Translation2d modulePos, String moduleName, int moduleNum, int driveMotorPort, int turnMotorPort, double zeroOffset) {
        m_modulePos = modulePos;
        m_moduleName = moduleName;
        m_moduleNum = moduleNum;

        m_turnMotor = new CANSparkMax(turnMotorPort, MotorType.kBrushless);
        m_driveMotor = new TalonFX(driveMotorPort);

        m_turnMotor.setIdleMode(IdleMode.kCoast);
        m_driveMotor.setNeutralMode(NeutralModeValue.Coast);

        //m_turnEncoder = m_turnMotor.getEncoder();
        m_turnAbsEncoder = m_turnMotor.getAbsoluteEncoder(Type.kDutyCycle);

        m_turnPIDController = m_turnMotor.getPIDController();

        m_turnPIDController.setFeedbackDevice(m_turnAbsEncoder);

        // set turn PID values
        m_turnPIDController.setP(Constants.ModuleConstants.kTurnP);
        m_turnPIDController.setI(Constants.ModuleConstants.kTurnI);
        m_turnPIDController.setD(Constants.ModuleConstants.kTurnD);

        m_turnPIDController.setPositionPIDWrappingMinInput(0);
        m_turnPIDController.setPositionPIDWrappingMaxInput(2 * Math.PI);
        m_turnPIDController.setPositionPIDWrappingEnabled(true);

        m_turnAbsEncoder.setPositionConversionFactor(Constants.ModuleConstants.kModuleTurnPositionConversionFactor);

        // TODO: drive motor PID tune
        TalonFXConfiguration driveMotorConfiguration = new TalonFXConfiguration();
        driveMotorConfiguration.Slot0.kP = Constants.ModuleConstants.kDriveP;
        driveMotorConfiguration.Slot0.kI = Constants.ModuleConstants.kDriveI;
        driveMotorConfiguration.Slot0.kD = Constants.ModuleConstants.kDriveD;
        driveMotorConfiguration.Feedback.RotorToSensorRatio = 1;
        driveMotorConfiguration.Feedback.SensorToMechanismRatio = 8.14;
        // driveMotorConfiguration.Feedback.SensorToMechanismRatio = Constants.ModuleConstants.kDrivePositionConversionFactor;

        m_driveMotor.getConfigurator().apply(driveMotorConfiguration);

        m_turnPIDController.setOutputRange(-1, 1);

        //m_turnAbsEncoder.setZeroOffset(zeroOffset);
        // m_turnPIDController.setReference(Units.degreesToRadians(90), ControlType.kPosition);

        m_rpsLog = new LoggableDouble(moduleName + "rps", true);
        m_desiredSpeedLog = new LoggableDouble(moduleName + "ms DesiredSpeed", true);
        m_currentSpeedLog = new LoggableDouble(moduleName + "ms CurrentSpeed", true, true, () -> getState().speedMetersPerSecond);


        Logger.getInstance().addLoggable(m_rpsLog);
        Logger.getInstance().addLoggable(m_desiredSpeedLog);
        Logger.getInstance().addLoggable(m_currentSpeedLog);
    }

    public Translation2d getPos() {
        return m_modulePos;
    }
    
    public void setDesiredState(SwerveModuleState state, boolean optimize) {
        SwerveModuleState optimizedState = optimize ? RevOptimizer.optimize(state, Rotation2d.fromRadians(m_turnAbsEncoder.getPosition())) : state;
        
        double rps = optimizedState.speedMetersPerSecond * (1 / (2 * Math.PI * Constants.ModuleConstants.kWheelRadius));
        m_desiredSpeedLog.log(optimizedState.speedMetersPerSecond);
        m_rpsLog.log(rps);
        m_driveMotor.setControl(new VelocityVoltage(rps));
        // m_turnPIDController.setReference(Units.degreesToRadians(90), ControlType.kPosition);
        m_turnPIDController.setReference(optimizedState.angle.getRadians(), ControlType.kPosition);
    }


    public void setDesiredState(SwerveModuleState state) {
        setDesiredState(state, true);
    }

    public void brakeAll() {
        m_turnMotor.setIdleMode(IdleMode.kBrake);
        m_driveMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void coastAll() {
        m_turnMotor.setIdleMode(IdleMode.kCoast);
        m_driveMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public String getName() {
        return m_moduleName;
    }

    public int getNumber() {
        return m_moduleNum;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(Conversions.wheelToMeters(m_driveMotor.getVelocity().getValueAsDouble()), Rotation2d.fromRadians(m_turnAbsEncoder.getPosition()));
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(Conversions.wheelToMeters(m_driveMotor.getPosition().getValueAsDouble()), Rotation2d.fromRadians(m_turnAbsEncoder.getPosition()));
    }
}
