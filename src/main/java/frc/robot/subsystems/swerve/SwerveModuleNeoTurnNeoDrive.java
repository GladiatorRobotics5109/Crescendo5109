package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.AccelStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.RevOptimizer;

/** 
 * Represents a swerve module with a NEO (SparkMAX) turn motor and a NEO (SparkMAX) drive motor.
 */
public class SwerveModuleNeoTurnNeoDrive {
    private final Translation2d m_modulePos;
    private final String m_moduleName;
    private final int m_moduleNum;

    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_turnMotor;

    private final RelativeEncoder m_driveEncoder;
    private final RelativeEncoder m_turnEncoder;

    private final SparkPIDController m_drivePIDController;
    private final SparkPIDController m_turnPIDController;

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

        m_drivePIDController.setP(Constants.ModuleConstants.kDriveP);
        m_drivePIDController.setI(Constants.ModuleConstants.kDriveI);
        m_drivePIDController.setD(Constants.ModuleConstants.kDriveD);

        m_turnPIDController.setP(Constants.ModuleConstants.kTurnP);
        m_turnPIDController.setI(Constants.ModuleConstants.kTurnI);
        m_turnPIDController.setD(Constants.ModuleConstants.kTurnD);

        m_turnPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

        m_turnPIDController.setSmartMotionMaxAccel(Constants.SwerveConstants.kMaxAngularSpeed, 0);
        m_turnPIDController.setSmartMotionMaxVelocity(Constants.SwerveConstants.kMaxAngularSpeed, 0);

        m_driveEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kDriveVelocityConversionFactor);
        m_driveEncoder.setPositionConversionFactor(Constants.ModuleConstants.kDrivePositionConversionFactor);

        m_turnEncoder.setPositionConversionFactor(Constants.ModuleConstants.kTurnPositionConversionFactor);
        m_turnEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kTurnVelocityConversionFactor);

        m_turnPIDController.setOutputRange(-1, 1);
    }



    
    public Translation2d getPos() {
        return m_modulePos;
    }

    
    public void setDesiredState(SwerveModuleState state, boolean optimize) {
        SwerveModuleState optimizedState = optimize ? RevOptimizer.optimize(state, new Rotation2d(m_turnEncoder.getPosition())) : state;
        
        m_drivePIDController.setReference(optimizedState.speedMetersPerSecond, ControlType.kVelocity);
        m_turnPIDController.setReference(optimizedState.angle.getRadians(), ControlType.kPosition);
    }

    
    public void setDesiredState(SwerveModuleState state) {
        setDesiredState(state, true);
    }

    
    public void brakeAll() {
        m_driveMotor.setIdleMode(IdleMode.kBrake);
        m_turnMotor.setIdleMode(IdleMode.kBrake);
    }

    
    public void coastAll() {
        m_driveMotor.setIdleMode(IdleMode.kCoast);
        m_turnMotor.setIdleMode(IdleMode.kCoast);
    }

    
    public String getName() {
        return m_moduleName;
    }

    
    public int getNumber() {
        return m_moduleNum;
    }

    
    public void resetEncoders() {
        m_driveEncoder.setPosition(0.0);
        m_turnEncoder.setPosition(0.0);
    }

    
    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveEncoder.getVelocity(), Rotation2d.fromRadians(m_turnEncoder.getPosition()));
    }

    
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(m_driveEncoder.getPosition(), Rotation2d.fromRadians(m_turnEncoder.getPosition()));
    }

    public double getDrivePos() {
        return m_driveEncoder.getPosition();
    }

    public double getTurnPos() {
        return m_turnEncoder.getPosition();
    }
}
