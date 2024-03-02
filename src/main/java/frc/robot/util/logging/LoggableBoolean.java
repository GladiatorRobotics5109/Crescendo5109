package frc.robot.util.logging;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.BooleanLogEntry;

public final class LoggableBoolean extends Loggable<Boolean> {
    private BooleanSupplier m_valueSuupplier;
    private BooleanPublisher m_publisher;
    private BooleanLogEntry m_wpiEntry;

    public LoggableBoolean(
        String name, 
        boolean logToNetworkTables,
        boolean liveLog,
        BooleanSupplier valueSupplier) {
        super(name, logToNetworkTables, liveLog);

        m_valueSuupplier = valueSupplier;

        m_publisher = NetworkTableInstance.getDefault().getBooleanTopic(name).publish();

        m_wpiEntry = new BooleanLogEntry(m_wpiLog, name);
    }

    public LoggableBoolean(String name, boolean logToNetworkTables) {
        super(name, logToNetworkTables);

        m_valueSuupplier = null;

        m_publisher = NetworkTableInstance.getDefault().getBooleanTopic(name).publish();

        m_wpiEntry = new BooleanLogEntry(m_wpiLog, name);
    }

    public LoggableBoolean(String name) {
        super(name);

        m_valueSuupplier = null;

        m_publisher = NetworkTableInstance.getDefault().getBooleanTopic(name).publish();

        m_wpiEntry = new BooleanLogEntry(m_wpiLog, name);
    }

    @Override
    public void log(Boolean value) {
        if (m_logToNetworkTables) {
            m_publisher.set(value);
        }

        m_wpiEntry.append(value);
    }

    @Override
    protected void log() {
        if(m_liveLog)
            log(m_valueSuupplier.getAsBoolean());
    }
}
