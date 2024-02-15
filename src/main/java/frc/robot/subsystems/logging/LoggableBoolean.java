package frc.robot.subsystems.logging;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LoggableBoolean extends Loggable<Boolean> {
    private BooleanSupplier m_valueSuupplier;
    private BooleanPublisher m_publisher;

    public LoggableBoolean(
        String subsystem, 
        String name, 
        boolean logToNetworkTables,
        boolean liveLog,
        BooleanSupplier valueSupplier) {
        super(subsystem, name, logToNetworkTables, liveLog);

        m_valueSuupplier = valueSupplier;

        m_publisher = NetworkTableInstance.getDefault().getBooleanTopic(name).publish();
    }

    @Override
    public void log(Boolean value) {
        if (m_logToNetworkTables) {
            m_publisher.set(value);
        }

        m_wpiLog.appendBoolean(m_wpiEntry, value, 0);
    }

    @Override
    public void log() {
        if(m_liveLog)
            log(m_valueSuupplier.getAsBoolean());
    }
}
