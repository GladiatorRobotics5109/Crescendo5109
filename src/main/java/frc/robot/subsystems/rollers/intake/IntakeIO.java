package frc.robot.subsystems.rollers.intake;

import frc.robot.util.GenericMotorInputs;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public class IntakeIOInputs extends GenericMotorInputs {

    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setVoltage(double volts) {}

    public default void stop() {
        setVoltage(0);
    }
}
