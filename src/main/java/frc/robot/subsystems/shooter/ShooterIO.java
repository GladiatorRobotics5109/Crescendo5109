package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs {
        public double leftMotorPositionRad = Double.NaN;
        public double leftMotorVelocityRadPerSec = Double.NaN;
        public double leftMotorAppliedVolts = Double.NaN;
        public double leftMotorSupplyCurrentAmps = Double.NaN;
        public double leftMotorTempCelsius = Double.NaN;

        public double rightMotorPositionRad = Double.NaN;
        public double rightMotorVelocityRadPerSec = Double.NaN;
        public double rightMotorAppliedVolts = Double.NaN;
        public double rightMotorSupplyCurrentAmps = Double.NaN;
        public double rightMotorTempCelsius = Double.NaN;
    }

    public default void updateInputs(ShooterIOInputs inputs) {};

    public default void setVoltage(double leftVolts, double rightVolts) {};
}
