package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
    private final WPI_TalonFX m_turnMotor;

    private final RelativeEncoder m_driveEncoder;

    private final SparkMaxPIDController m_drivePIDController;


    public SwerveModuleKrakenTurnNeoDrive(Translation2d modulePos, String moduleName, int moduleNum, int driveMotorPort, int turnMotorPort) {
        m_modulePos = modulePos;
        m_moduleName = moduleName;
        m_moduleNum = moduleNum;

        m_driveMotor = new CANSparkMax(driveMotorPort, MotorType.kBrushless);
        m_turnMotor = new WPI_TalonFX(turnMotorPort);

        m_driveMotor.setIdleMode(IdleMode.kCoast);
        m_turnMotor.setNeutralMode(NeutralMode.Coast);

        m_driveEncoder = m_driveMotor.getEncoder();

        m_drivePIDController = m_driveMotor.getPIDController();

        // set drive PID values
        m_drivePIDController.setP(0.3);
        m_drivePIDController.setI(0.0);
        m_drivePIDController.setD(0.0);

        // TODO: turn motor PID tune
        m_turnMotor.config_kP(0, 1.5);
        m_turnMotor.config_kI(0, 0.0);
        m_turnMotor.config_kD(0, 0.0);
        
        m_driveEncoder.setVelocityConversionFactor(1 / (Constants.SwerveConstants.kNeoTicksPerWheelRadian) * Constants.SwerveConstants.kWheelRadius);
        m_driveEncoder.setPositionConversionFactor(Constants.SwerveConstants.kNeoTicksPerMotorRevolution / (Constants.SwerveConstants.kNeoTicksPerWheelRadian) * Constants.SwerveConstants.kWheelRadius);
        // unlike the SparkMAXes, we have to run kraken ticks -> radian conversion manually in our code.
        // m_turnMotor.getSelectedSensorPosition() should never really be used in this class to get encoder position
        // getTurnEncoderPositionRad() should be used instead.
    }

    @Override
    public Translation2d getPoseRelative() {
        return m_modulePos;
    }

    @Override
    public void setDesiredState(SwerveModuleState state) {
        SwerveModuleState optimizedState = RevOptimizer.optimize(state, Rotation2d.fromRadians(getTurnWheelPositionRad()));

        m_drivePIDController.setReference(optimizedState.speedMetersPerSecond, ControlType.kVelocity);
        m_turnMotor.set(ControlMode.Position, Conversions.radToKraken(state.angle.getRadians(), 1 / Constants.SwerveConstants.kSwerveTurnGearRatio));
    }

    @Override
    public SwerveModulePosition getModulePose() {
        return new SwerveModulePosition(-m_driveEncoder.getPosition(), Rotation2d.fromRadians(getTurnWheelPositionRad()));
    }

    @Override
    public void brakeAll() {
        m_driveMotor.setIdleMode(IdleMode.kBrake);
        m_turnMotor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void coastAll() {
        m_driveMotor.setIdleMode(IdleMode.kCoast);
        m_turnMotor.setNeutralMode(NeutralMode.Coast);
    }

    @Override
    public String getName() {
        return m_moduleName;
    }

    @Override
    public int getNumber() {
        return m_moduleNum;
    }

    // do nothing because turn encoder is absolute
    @Override
    public void resetTurnEncoder() {}

    private double getTurnWheelPositionRad() {
        // TODO: test this conversion, probably the one not commented out is correct
        return Conversions.krakenToRad(m_turnMotor.getSelectedSensorPosition(), Constants.SwerveConstants.kSwerveTurnGearRatio);
    }
}
