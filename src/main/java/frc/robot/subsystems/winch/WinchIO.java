package frc.robot.subsystems.winch;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface WinchIO {
    @AutoLog
    class WinchIOInputs {
        public double motorPositionRad = Double.NaN;
        public double motorVelocityRadPerSec = Double.NaN;

        public double motorAppliedVolts = Double.NaN;
        public double motorSupplyCurrentAmps = Double.NaN;
        public double motorTempCelsius = Double.NaN;
    }

    public default void updateInputs(WinchIOInputs inputs) {};

    public default void setVoltage(double volts) {};

    public default void setMotorPosition(Rotation2d angle) {};
}
