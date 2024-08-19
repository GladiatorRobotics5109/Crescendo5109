package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.util.Conversions;

public class ShooterIOSparkMax implements ShooterIO {
    private final CANSparkMax m_left;
    private final CANSparkMax m_right;

    private final RelativeEncoder m_leftEncoder;
    private final RelativeEncoder m_rightEncoder;

    public ShooterIOSparkMax(int leftMotorPort, int rightMotorPort) {
        m_left = new CANSparkMax(leftMotorPort, MotorType.kBrushless);
        m_right = new CANSparkMax(rightMotorPort, MotorType.kBrushless);

        m_leftEncoder = m_left.getEncoder();
        m_rightEncoder = m_right.getEncoder();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftMotorPositionRad = Conversions.rotationsToRadians(m_leftEncoder.getPosition());
        inputs.leftMotorVelocityRadPerSec = Conversions.rotationsPerMinuteToRadiansPerSecond(
            m_leftEncoder.getVelocity()
        );
        inputs.leftMotorAppliedVolts = m_left.getAppliedOutput() * m_left.getBusVoltage();
        inputs.leftMotorSupplyCurrentAmps = m_left.getOutputCurrent();
        inputs.leftMotorTempCelsius = m_left.getMotorTemperature();

        inputs.rightMotorPositionRad = Conversions.rotationsToRadians(m_rightEncoder.getPosition());
        inputs.rightMotorVelocityRadPerSec = Conversions.rotationsPerMinuteToRadiansPerSecond(
            m_rightEncoder.getVelocity()
        );
        inputs.rightMotorAppliedVolts = m_right.getAppliedOutput() * m_right.getBusVoltage();
        inputs.rightMotorSupplyCurrentAmps = m_right.getOutputCurrent();
        inputs.rightMotorTempCelsius = m_right.getMotorTemperature();
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        m_left.setVoltage(leftVolts);
        m_right.setVoltage(rightVolts);
    }
}
