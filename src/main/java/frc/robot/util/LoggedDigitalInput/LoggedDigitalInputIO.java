package frc.robot.util.LoggedDigitalInput;

import org.littletonrobotics.junction.AutoLog;

public interface LoggedDigitalInputIO {
    @AutoLog
    public static class LoggedDigitalInputIOInputs {
        public boolean value;
        public int channel;
        public int analogTriggerTypeForRouting;
        public boolean isAnalogTrigger;
        public int portHandleForRouting;
    }

    public default void updateInputs(LoggedDigitalInputIOInputs inputs) {}

    public default void close() {}
}
