package frc.robot.util.LoggedDigitalInput;

public class LoggedDigitalInputIOSim implements LoggedDigitalInputIO {

    public LoggedDigitalInputIOSim() {}

    @Override
    public void updateInputs(LoggedDigitalInputIOInputs inputs) {
        inputs.value = false;
        inputs.channel = 0;
        inputs.analogTriggerTypeForRouting = 0;
        inputs.isAnalogTrigger = false;
        inputs.portHandleForRouting = 0;
    }
}
