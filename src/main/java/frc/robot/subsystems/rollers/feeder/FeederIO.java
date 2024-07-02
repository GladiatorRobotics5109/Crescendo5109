package frc.robot.subsystems.rollers.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    @AutoLog
    public class FeederIOInputs {
        public double feederPositionRad = Double.NaN;
        public double feederVelocityRadPerSec = Double.NaN;
        public double motorAppliedVolts = Double.NaN;
        public double motorSupplyCurrentAmps = Double.NaN;
        public double motorTempCelcius = Double.NaN;
    }

    public default void updateInputs(FeederIOInputs inputs) {}

    public default void setVoltage(double volts) {}

    public default void stop() {
        setVoltage(0);
    }
}
