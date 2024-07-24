package frc.robot.util.LoggedDigitalInput;

import frc.robot.Constants;
import frc.robot.util.periodic.PeriodicUtil;
import org.littletonrobotics.junction.Logger;

public class LoggedDigitalInput extends PeriodicUtil.Periodic {
    private final LoggedDigitalInputIO m_io;
    private final LoggedDigitalInputIOInputsAutoLogged m_inputs;

    private final String m_name;

    public LoggedDigitalInput(String name, int channel) {
        m_name = name;
        switch (Constants.kCurrentMode) {
            case REAL:
                m_io = new LoggedDigitalInputIOReal(channel);

                break;
            case SIM:
                // fix supplier
                m_io = new LoggedDigitalInputIOSim();

                break;
            default:
                m_io = new LoggedDigitalInputIO() {};

                break;
        }

        m_inputs = new LoggedDigitalInputIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs(m_name, m_inputs);
    }
}
