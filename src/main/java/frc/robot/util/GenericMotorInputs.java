package frc.robot.util;

public abstract class GenericMotorInputs {
    public double motorPositionRad = Double.NaN;
    public double motorVelocityRadPerSec = Double.NaN;
    public double motorAppliedVolts = Double.NaN;
    public double[] motorSupplyCurrentAmps = new double[] {};
    public double motorTempCelcius = Double.NaN;
}
