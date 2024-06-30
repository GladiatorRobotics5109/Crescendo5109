package frc.robot.subsystems.rollers.feeder;

import frc.robot.util.GenericMotorInputs;
import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    @AutoLog
    public class FeederIOInputs extends GenericMotorInputs {

    }

    public default void updateInputs(FeederIOInputs inputs) {}

    public default void setVoltage(double volts) {}
}
