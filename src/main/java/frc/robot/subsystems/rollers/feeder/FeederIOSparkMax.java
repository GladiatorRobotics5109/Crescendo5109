package frc.robot.subsystems.rollers.feeder;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.RollersConstants.FeederConstants;
import frc.robot.util.Conversions;

public class FeederIOSparkMax implements FeederIO {
    private CANSparkMax m_motor;
    private RelativeEncoder m_encoder;

    public FeederIOSparkMax(int motorPort) {
        m_motor = new CANSparkMax(motorPort, MotorType.kBrushless);

        m_motor.restoreFactoryDefaults();

        m_motor.setSmartCurrentLimit(FeederConstants.kRealCurrentLimitAmps);
        m_motor.setIdleMode(IdleMode.kCoast);

        m_encoder = m_motor.getEncoder();
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.feederPositionRad = m_encoder.getPosition();
        inputs.feederVelocityRadPerSec = Conversions.rotationsPerMinuteToRadiansPerSecond(m_encoder.getVelocity());
        inputs.motorAppliedVolts = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        inputs.motorSupplyCurrentAmps = m_motor.getOutputCurrent();
        inputs.motorTempCelcius = m_motor.getMotorTemperature();
    }

    @Override
    public void setVoltage(double volts) {
        m_motor.setVoltage(volts);
    }
}
