package frc.robot.subsystems.winch;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.WinchConstants;
import frc.robot.util.Conversions;

public class WinchIOSparkMax implements WinchIO {
    private CANSparkMax m_motor;
    private RelativeEncoder m_encoder;

    public WinchIOSparkMax(int motorPort) {
        m_motor = new CANSparkMax(motorPort, MotorType.kBrushless);

        m_motor.restoreFactoryDefaults();

        m_motor.setIdleMode(IdleMode.kBrake);

        m_motor.setSmartCurrentLimit(WinchConstants.kRealMotorCurrentLimit);

        m_motor.burnFlash();

        m_encoder = m_motor.getEncoder();
    }

    @Override
    public void updateInputs(WinchIOInputs inputs) {
        inputs.motorPositionRads = Conversions.rotationsToRadians(m_encoder.getPosition());
        inputs.motorVelocityRadsPerSec = Conversions.rotationsPerMinuteToRadiansPerSecond(m_encoder.getVelocity());

        inputs.motorAppliedVolts = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        inputs.motorSupplyCurrentAmps = m_motor.getOutputCurrent();
        inputs.motorTempCelsius = m_motor.getMotorTemperature();
    }

    @Override
    public void setVoltage(double volts) {
        m_motor.setVoltage(volts);
    }
}
