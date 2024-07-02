package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ShooterIOSim implements ShooterIO {
    private DCMotorSim m_leftMotor;
    private DCMotorSim m_rightMotor;

    public ShooterIOSim() {
        m_leftMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 0.0015);
        m_rightMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 0.0015);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        m_leftMotor.update(Constants.kRobotLoopPeriodSecs);
        m_rightMotor.update(Constants.kRobotLoopPeriodSecs);

        inputs.leftMotorPositionRad = m_leftMotor.getAngularPositionRad();
        inputs.leftMotorVelocityRadPerSec = m_leftMotor.getAngularVelocityRadPerSec();
        inputs.leftMotorAppliedVolts = inputs.leftMotorAppliedVolts;
        inputs.leftMotorSupplyCurrentAmps = m_leftMotor.getCurrentDrawAmps();
        inputs.leftMotorTempCelsius = inputs.leftMotorTempCelsius;

        inputs.rightMotorPositionRad = m_rightMotor.getAngularPositionRad();
        inputs.rightMotorVelocityRadPerSec = m_rightMotor.getAngularVelocityRadPerSec();
        inputs.rightMotorAppliedVolts = inputs.rightMotorAppliedVolts;
        inputs.rightMotorSupplyCurrentAmps = m_rightMotor.getCurrentDrawAmps();
        inputs.rightMotorTempCelsius = inputs.rightMotorTempCelsius;
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        m_leftMotor.setInputVoltage(MathUtil.clamp(leftVolts, -12.0, 12.0));
        m_rightMotor.setInputVoltage(MathUtil.clamp(rightVolts, -12.0, 12.0));
    }
}
