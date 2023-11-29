package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.RevOptimizer;

/** 
 * Represents a swerve module with a NEO (SparkMAX) turn motor and a NEO (SparkMAX) drive motor.
 */
public class SwerveModuleNeoTurnNeoDrive extends SwerveModule {
    private final Translation2d m_modulePos;
    private final String m_moduleName;
    private final int m_moduleNum;

    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_turnMotor;

    private final RelativeEncoder m_driveEncoder;
    private final RelativeEncoder m_turnEncoder;

    private final SparkMaxPIDController m_drivePIDController;
    private final SparkMaxPIDController m_turnPIDController;

    public SwerveModuleNeoTurnNeoDrive(Translation2d modulePos, String moduleName, int moduleNum, int driveMotorPort, int turnMotorPort) {
        m_modulePos = modulePos;
        m_moduleName = moduleName;
        m_moduleNum = moduleNum;

        m_driveMotor = new CANSparkMax(driveMotorPort, MotorType.kBrushless);
        m_turnMotor = new CANSparkMax(turnMotorPort, MotorType.kBrushless);

        m_driveMotor.setIdleMode(IdleMode.kCoast);
        m_turnMotor.setIdleMode(IdleMode.kCoast);

        m_driveEncoder = m_driveMotor.getEncoder();
        m_turnEncoder = m_turnMotor.getEncoder();

        m_drivePIDController = m_driveMotor.getPIDController();
        m_turnPIDController = m_turnMotor.getPIDController();

        m_drivePIDController.setP(0.3);
        m_drivePIDController.setI(0.0);
        m_drivePIDController.setD(0.0);

        m_turnPIDController.setP(1.5);
        m_turnPIDController.setI(0.0);
        m_turnPIDController.setD(0.0);

        m_turnPIDController.setSmartMotionAccelStrategy(com.revrobotics.SparkMaxPIDController.AccelStrategy.kTrapezoidal, 0);
        m_turnPIDController.setSmartMotionMaxAccel(Constants.SwerveConstants.kMaxAngularSpeed, 0);
        m_turnPIDController.setSmartMotionMaxVelocity(Constants.SwerveConstants.kMaxAngularSpeed, 0);

        m_driveEncoder.setVelocityConversionFactor(1 / (Constants.SwerveConstants.kNeoTicksPerWheelRadian) * Constants.SwerveConstants.kWheelRadius);
        m_driveEncoder.setPositionConversionFactor(42 / (Constants.SwerveConstants.kNeoTicksPerWheelRadian) * Constants.SwerveConstants.kWheelRadius);
        // TODO: test swerve turn position working
        m_turnEncoder.setPositionConversionFactor(1 / Constants.SwerveConstants.kNeoTicksPerTurnWheelRadian);

        m_turnPIDController.setOutputRange(-Math.PI, Math.PI);
    }

    @Override
    public Translation2d getPos() {
        return m_modulePos;
    }

    @Override
    public void setDesiredState(SwerveModuleState state) {
        SwerveModuleState optimizedState = RevOptimizer.optimize(state, new Rotation2d(m_turnEncoder.getPosition()));
        
        m_drivePIDController.setReference(optimizedState.speedMetersPerSecond, ControlType.kVelocity);
        m_turnPIDController.setReference(optimizedState.angle.getRadians(), ControlType.kPosition);
    }

    @Override
    public void brakeAll() {
        m_driveMotor.setIdleMode(IdleMode.kBrake);
        m_turnMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void coastAll() {
        m_driveMotor.setIdleMode(IdleMode.kCoast);
        m_turnMotor.setIdleMode(IdleMode.kCoast);
    }

    @Override
    public String getName() {
        return m_moduleName;
    }

    @Override
    public int getNumber() {
        return m_moduleNum;
    }
}
