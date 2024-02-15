package frc.robot.subsystems.logging;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LoggableBoolean extends Loggable {
    private BooleanSupplier m_valueSuupplier;
    private BooleanPublisher m_publisher;

    public LoggableBoolean(
        String subsystem, 
        String name, 
        boolean logToNetworkTables, 
        BooleanSupplier valueSupplier) {
        super(subsystem, name, logToNetworkTables);

        m_valueSuupplier = valueSupplier;

        m_publisher = NetworkTableInstance.getDefault().getBooleanTopic(name).publish();
    }

    @Override
    public void Log() {
        boolean value = m_valueSuupplier.getAsBoolean();

        if (m_logToNetworkTables) {
            m_publisher.set(value);
        }

        m_wpiLog.appendBoolean(m_wpiEntry, value, 0);
    }
}
