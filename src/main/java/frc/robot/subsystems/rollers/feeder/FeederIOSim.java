package frc.robot.subsystems.rollers.feeder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class FeederIOSim implements FeederIO {
    private final DCMotorSim m_motor;

    public FeederIOSim() {
        m_motor = new DCMotorSim(DCMotor.getNEO(1), 1, 0.02);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        m_motor.update(Constants.kRobotLoopPeriodSecs);

        inputs.feederPositionRad = m_motor.getAngularPositionRad();
        inputs.feederVelocityRadPerSec = m_motor.getAngularVelocityRadPerSec();
        inputs.motorAppliedVolts = inputs.motorAppliedVolts;
        inputs.motorSupplyCurrentAmps = m_motor.getCurrentDrawAmps();
        inputs.motorTempCelcius = inputs.motorTempCelcius;
    }
}
