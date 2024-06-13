package frc.robot.subsystems.swerve.swerveModule;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.util.Conversions;

public class SwerveModuleIOTalonFXDriveSparkMaxTurn implements SwerveModuleIO {
    private final TalonFX m_driveMotor;
    private final CANSparkMax m_turnMotor;

    private final StatusSignal<Double> m_drivePosition;
    private final StatusSignal<Double> m_driveVelocity;
    private final StatusSignal<Double> m_driveAppliedVolts;
    private final StatusSignal<Double> m_driveCurrent;

    private final RelativeEncoder m_turnRelativeEncoder;
    private final AbsoluteEncoder m_turnAbsoluteEncoder;
    private final Rotation2d m_turnAbsoluteEncoderOffset;

    private final VoltageOut m_voltageOut;

    public SwerveModuleIOTalonFXDriveSparkMaxTurn(
        int driveMotorPort,
        int turnMotorPort,
        boolean invertTurnMotor,
        Rotation2d turnAbsoluteEncoderOffset
    ) {
        m_driveMotor = new TalonFX(driveMotorPort);

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        m_driveMotor.getConfigurator().apply(driveConfig);

        m_drivePosition = m_driveMotor.getPosition();
        m_driveVelocity = m_driveMotor.getVelocity();
        m_driveAppliedVolts = m_driveMotor.getMotorVoltage();
        m_driveCurrent = m_driveMotor.getSupplyCurrent();

        m_turnMotor = new CANSparkMax(turnMotorPort, CANSparkLowLevel.MotorType.kBrushless);

        m_turnMotor.restoreFactoryDefaults();

        m_turnMotor.setCANTimeout(250);

        m_turnRelativeEncoder = m_turnMotor.getEncoder();
        m_turnAbsoluteEncoder = m_turnMotor.getAbsoluteEncoder();
        m_turnAbsoluteEncoderOffset = turnAbsoluteEncoderOffset;

        m_turnMotor.setInverted(invertTurnMotor);
        m_turnMotor.setSmartCurrentLimit(30);
        m_turnMotor.enableVoltageCompensation(12.0);

        m_turnRelativeEncoder.setPosition(0.0);
        m_turnRelativeEncoder.setMeasurementPeriod(10);
        m_turnRelativeEncoder.setAverageDepth(2);

        m_turnMotor.setCANTimeout(0);

        m_turnMotor.setPeriodicFramePeriod(
            CANSparkLowLevel.PeriodicFrame.kStatus2,
            (int)(1000.0 / Constants.SwerveConstants.kOdometryFrequency)
        );

        m_voltageOut = new VoltageOut(0);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.drivePositionRad = Conversions.DriveMotorRotToDriveWheelRad(m_drivePosition.getValueAsDouble());
        inputs.driveVelocityRadPerSec = Conversions.DriveMotorRotToDriveWheelRad(m_driveVelocity.getValueAsDouble());
        inputs.driveAppliedVolts = m_driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = new double[] {
            m_driveCurrent.getValueAsDouble()
        };

        inputs.turnAbsolutePosition = Rotation2d.fromRotations(m_turnAbsoluteEncoder.getPosition())
            .minus(m_turnAbsoluteEncoderOffset);
        inputs.turnPosition = Conversions.TurnMotorRotToWheelRotation2d(m_turnRelativeEncoder.getPosition());
        inputs.turnVelocityRadPerSec = Conversions.TurnMotorRotToWheelRad(m_turnRelativeEncoder.getVelocity());
        inputs.turnAppliedVolts = m_turnMotor.getAppliedOutput() * m_turnMotor.getBusVoltage();
        inputs.turnCurrentAmps = new double[] {
            m_turnMotor.getOutputCurrent()
        };
    }

    @Override
    public void setDriveVoltage(double volts) {
        m_driveMotor.setControl(m_voltageOut.withOutput(volts));
    }

    @Override
    public void setTurnVoltage(double volts) {
        m_turnMotor.setVoltage(volts);
    }

    @Override
    public void setDriveBrakeMode(boolean enabled) {
        m_driveMotor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setTurnBrakeMode(boolean enabled) {
        m_turnMotor.setIdleMode(enabled ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
    }

}
