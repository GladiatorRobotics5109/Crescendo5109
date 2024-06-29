package frc.robot.subsystems.winch;

import org.littletonrobotics.junction.AutoLog;

public interface WinchIO {
    @AutoLog
    class WinchIOInputs {
        public double motorPositionRads = Double.NaN;
        public double motorVelocityRadsPerSec = Double.NaN;

        public double motorAppliedVolts = Double.NaN;
        public double motorSupplyCurrentAmps = Double.NaN;
        public double motorTorqueCurrentAmps = Double.NaN;
        public double motorTempCelsius = Double.NaN;
    }

    public default void updateInputs(WinchIOInputs inputs) {};

    public default void setVoltage(double volts) {};
}
