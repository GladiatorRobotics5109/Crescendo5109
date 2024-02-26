package frc.robot.util.logging;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;

public final class LoggableDouble extends Loggable<Double> {
    private DoubleSupplier m_valueSupplier;
    private DoublePublisher m_publisher;
    private DoubleLogEntry m_wpiEntry;

    public LoggableDouble(
        String name, 
        boolean logToNetworkTables,
        boolean liveLog,
        DoubleSupplier valueSupplier) {
        super(name, logToNetworkTables, liveLog);

        m_valueSupplier = valueSupplier;

        m_publisher = NetworkTableInstance.getDefault().getDoubleTopic(name).publish();

        m_wpiEntry = new DoubleLogEntry(m_wpiLog, name);
    }

    public LoggableDouble(String name, boolean logToNetworkTables) {
        super(name, logToNetworkTables);

        m_valueSupplier = null;

        m_publisher = NetworkTableInstance.getDefault().getDoubleTopic(name).publish();

        m_wpiEntry = new DoubleLogEntry(m_wpiLog, name);
    }

    public LoggableDouble(String name) {
        super(name);

        m_valueSupplier = null;

        m_publisher = NetworkTableInstance.getDefault().getDoubleTopic(name).publish();

        m_wpiEntry = new DoubleLogEntry(m_wpiLog, name);
    }

    @Override
    public void log(Double value) {
        if (m_logToNetworkTables) {
            m_publisher.set(value);
        }

        m_wpiEntry.append(value);
    }

    @Override
    protected void log() {
        if(m_liveLog)
            log(m_valueSupplier.getAsDouble());
    }
}
