package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.MotorLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.Constants;
import frc.robot.util.Conversions;
import frc.robot.util.RevOptimizer;

public class SwerveModuleKrakenTurnKrakenDrive implements SwerveModule {
    private final Translation2d m_modulePos;
    private final String m_moduleName;
    private final int m_moduleNum;

    private final TalonFX m_driveMotor;
    private final TalonFX m_turnMotor;

    private final VelocityDutyCycle m_driveVelocity;
    private final PositionDutyCycle m_turnPosition;

    private final StatusSignal<Double> m_drivePositionSignal;
    private final StatusSignal<Double> m_driveVelocitySignal;
    private final StatusSignal<Double> m_turnPositionSignal;

    public SwerveModuleKrakenTurnKrakenDrive(Translation2d modulePos, String moduleName, int moduleNum, int driveMotorPort, int turnMotorPort) {
        m_modulePos = modulePos;
        m_moduleName = moduleName;
        m_moduleNum = moduleNum;

        m_turnMotor = new TalonFX(turnMotorPort);
        m_driveMotor = new TalonFX(driveMotorPort);

        m_turnMotor.setNeutralMode(NeutralModeValue.Coast);
        m_driveMotor.setNeutralMode(NeutralModeValue.Coast);

        TalonFXConfiguration turnConfig = new TalonFXConfiguration();
        turnConfig.CurrentLimits.SupplyCurrentLimit = 40;
        turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turnConfig.Slot0.kP = Constants.ModuleConstants.kTurnP;
        turnConfig.Slot0.kI = Constants.ModuleConstants.kTurnI;
        turnConfig.Slot0.kD = Constants.ModuleConstants.kTurnD;
        turnConfig.Slot0.kS = Constants.ModuleConstants.kTurnS;
        turnConfig.Feedback.SensorToMechanismRatio = 1 / Constants.ModuleConstants.kSwerveTurnGearRatio;

        m_turnMotor.getConfigurator().apply(turnConfig);

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.CurrentLimits.SupplyCurrentLimit = 40;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.Slot0.kP = Constants.ModuleConstants.kDriveP;
        driveConfig.Slot0.kI = Constants.ModuleConstants.kDriveI;
        driveConfig.Slot0.kD = Constants.ModuleConstants.kDriveD;
        driveConfig.Feedback.SensorToMechanismRatio = 1 / Constants.ModuleConstants.kSwerveDriveGearRatio;

        m_driveMotor.getConfigurator().apply(driveConfig);

        m_driveVelocity = new VelocityDutyCycle(0);
        m_turnPosition = new PositionDutyCycle(0);
        
        m_drivePositionSignal = m_driveMotor.getPosition();
        m_driveVelocitySignal = m_driveMotor.getVelocity();
        m_turnPositionSignal = m_turnMotor.getPosition();
    }

    @Override
    public Translation2d getPos() {
        return m_modulePos;
    }

    @Override
    public void setDesiredState(SwerveModuleState state, boolean optimize) {
        SwerveModuleState optimizedState = optimize ? RevOptimizer.optimize(state, getAngle()) : state;

        m_driveMotor.setControl(m_driveVelocity.withVelocity(optimizedState.speedMetersPerSecond));
        m_turnMotor.setControl(m_turnPosition.withPosition(optimizedState.angle.getRotations()));
    }

    @Override
    public void setDesiredState(SwerveModuleState state) {
        setDesiredState(state, true);
    }

    @Override
    public void brakeAll() {
        m_driveMotor.setNeutralMode(NeutralModeValue.Brake);
        m_turnMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void coastAll() {
        m_driveMotor.setNeutralMode(NeutralModeValue.Coast);
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
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Conversions.wheelRotToWheelM(m_driveVelocitySignal.getValueAsDouble()),
            Rotation2d.fromRotations(m_turnPositionSignal.getValueAsDouble())
        );
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            Conversions.wheelRotToWheelM(m_drivePositionSignal.getValueAsDouble()),
            Rotation2d.fromRotations(m_turnPositionSignal.getValueAsDouble())
        );
    }

    public SysIdRoutine getTurnSysIdRoutine(SwerveSubsystem swerve) {
        return new SysIdRoutine(
            new SysIdRoutine.Config(), 
            new SysIdRoutine.Mechanism(volts -> m_turnMotor.setVoltage(volts.in(Units.Volts)), log -> {
                    MotorLog motorLog = log.motor(m_moduleName + "/turnMotor");
                    motorLog.angularPosition(Units.Rotations.of(m_turnMotor.getPosition().getValueAsDouble()));
                    motorLog.angularVelocity(Units.RotationsPerSecond.of(m_turnMotor.getVelocity().getValueAsDouble()));
                    motorLog.voltage(Units.Volts.of(m_turnMotor.getSupplyVoltage().getValueAsDouble()));

                    motorLog.angularAcceleration(
                        Units.RadiansPerSecond.per(Units.Second).of(
                            m_turnMotor.getAcceleration().getValueAsDouble()
                        )
                    );
                },
                swerve
            )
        );
    }
}
