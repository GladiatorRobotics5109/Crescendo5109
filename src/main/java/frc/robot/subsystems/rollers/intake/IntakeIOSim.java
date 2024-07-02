package frc.robot.subsystems.rollers.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.RollersConstants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {
    private final DCMotorSim m_motor;

    public IntakeIOSim() {
        m_motor = new DCMotorSim(DCMotor.getNEO(1), IntakeConstants.kIntakeGearRatio, 0.02);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        m_motor.update(Constants.kRobotLoopPeriodSecs);

        inputs.intakePositionRad = m_motor.getAngularPositionRad();
        inputs.intakeVelocityRadPerSec = m_motor.getAngularVelocityRadPerSec();
        inputs.motorAppliedVolts = inputs.motorAppliedVolts;
        inputs.motorSupplyCurrentAmps = m_motor.getCurrentDrawAmps();
        inputs.motorTempCelcius = inputs.motorTempCelcius;
    }

    @Override
    public void setVoltage(double volts) {
        m_motor.setInputVoltage(volts);
    }
}
