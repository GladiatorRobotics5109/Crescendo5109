package frc.robot.subsystems.winch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.WinchConstants;

public class WinchIOSim implements WinchIO {
    private DCMotorSim m_motor;

    public WinchIOSim() {
        // TODO: figure out correct moi
        m_motor = new DCMotorSim(DCMotor.getNEO(4), WinchConstants.kSimWinchGearRatio, 0.0000001);
    }

    @Override
    public void updateInputs(WinchIOInputs inputs) {
        m_motor.update(Constants.kRobotLoopPeriodSecs);

        inputs.motorPositionRad = m_motor.getAngularPositionRad();
        inputs.motorVelocityRadPerSec = m_motor.getAngularVelocityRadPerSec();

        inputs.motorAppliedVolts = inputs.motorAppliedVolts;
        inputs.motorSupplyCurrentAmps = m_motor.getCurrentDrawAmps();
        inputs.motorTempCelsius = inputs.motorTempCelsius;
    }

    @Override
    public void setVoltage(double volts) {
        m_motor.setInputVoltage(MathUtil.clamp(-volts, -12.0, 12.0));
    }

    @Override
    public void setMotorPosition(Rotation2d angle) {
        m_motor.setState(angle.getRadians(), 0);
    }
}
