package frc.robot.subsystems.rollers.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.RollersConstants.IntakeConstants;
import frc.robot.util.Conversions;

public class IntakeIOSparkMax implements IntakeIO {
    private CANSparkMax m_motor;
    private RelativeEncoder m_encoder;

    public IntakeIOSparkMax(int motorPort) {
        m_motor = new CANSparkMax(motorPort, MotorType.kBrushless);

        m_motor.restoreFactoryDefaults();

        m_motor.setSmartCurrentLimit(IntakeConstants.kRealCurrentLimitAmps);
        m_motor.setIdleMode(IdleMode.kCoast);

        m_encoder = m_motor.getEncoder();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.motorPositionRad = m_encoder.getPosition();
        inputs.motorVelocityRadPerSec = Conversions.rotationsPerMinuteToRadiansPerSecond(m_encoder.getVelocity());
        inputs.motorAppliedVolts = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        inputs.motorSupplyCurrentAmps = new double[] {
            m_motor.getOutputCurrent()
        };
        inputs.motorTempCelcius = m_motor.getMotorTemperature();
    }

    @Override
    public void setVoltage(double volts) {
        m_motor.setVoltage(volts);
    }
}
