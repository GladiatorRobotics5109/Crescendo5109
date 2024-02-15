package frc.robot.subsystems.logging;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LoggableDouble extends Loggable<Double> {
    private DoubleSupplier m_valueSupplier;
    private DoublePublisher m_publisher;

    public LoggableDouble(
        String subsystem, 
        String name, 
        boolean logToNetworkTables,
        boolean liveLog,
        DoubleSupplier valueSupplier) {
        super(subsystem, name, logToNetworkTables, liveLog);

        m_valueSupplier = valueSupplier;

        m_publisher = NetworkTableInstance.getDefault().getDoubleTopic(name).publish();
    }
    public LoggableDouble(
        String subsystem,
        String name,
        DoubleSupplier valueSupplier) {
        super(subsystem, name, true, true);

        m_valueSupplier = valueSupplier;

        m_publisher = NetworkTableInstance.getDefault().getDoubleTopic(name).publish();
    }

    @Override
    public void log(Double value) {
        if (m_logToNetworkTables) {
            m_publisher.set(value);
        }

        m_wpiLog.appendDouble(m_wpiEntry, value, 0);
    }

    @Override
    public void log() {
        if(m_liveLog)
            log(m_valueSupplier.getAsDouble());
    }
}
