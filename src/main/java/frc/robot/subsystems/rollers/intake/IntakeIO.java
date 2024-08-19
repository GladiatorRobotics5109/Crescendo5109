package frc.robot.subsystems.rollers.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double intakePositionRad = Double.NaN;
        public double intakeVelocityRadPerSec = Double.NaN;
        public double motorAppliedVolts = Double.NaN;
        public double motorSupplyCurrentAmps = Double.NaN;
        public double motorTempCelsius = Double.NaN;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setVoltage(double volts) {}

    public default void stop() {
        setVoltage(0);
    }
}
