package frc.robot.util.LoggedDigitalInput;

import edu.wpi.first.wpilibj.DigitalInput;

public class LoggedDigitalInputIOReal implements LoggedDigitalInputIO {
    private final DigitalInput m_digitalInput;

    public LoggedDigitalInputIOReal(int channel) {
        m_digitalInput = new DigitalInput(channel);
    }

    @Override
    public void updateInputs(LoggedDigitalInputIOInputs inputs) {
        inputs.value = m_digitalInput.get();
        inputs.channel = m_digitalInput.getChannel();
        inputs.analogTriggerTypeForRouting = m_digitalInput.getAnalogTriggerTypeForRouting();
        inputs.isAnalogTrigger = m_digitalInput.isAnalogTrigger();
        inputs.portHandleForRouting = m_digitalInput.getPortHandleForRouting();

    }

    @Override
    public void close() {
        m_digitalInput.close();
    }
}
