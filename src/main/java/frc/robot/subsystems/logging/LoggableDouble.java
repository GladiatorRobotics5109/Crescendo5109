package frc.robot.subsystems.logging;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LoggableDouble extends Loggable {
    private DoubleSupplier m_valueSupplier;
    private DoublePublisher m_publisher;

    public LoggableDouble(
        String subsystem, 
        String name, 
        boolean logToNetworkTables, 
        DoubleSupplier valueSupplier, 
        DoublePublisher publisher) {
        super(subsystem, name, logToNetworkTables);

        m_valueSupplier = valueSupplier;

        m_publisher = NetworkTableInstance.getDefault().getDoubleTopic(name).publish();
    }
    public LoggableDouble(
        String subsystem,
        String name,
        DoubleSupplier valueSupplier,
        DoublePublisher publisher) {
        super(subsystem, name, true);

        m_valueSupplier = valueSupplier;

        m_publisher = NetworkTableInstance.getDefault().getDoubleTopic(name).publish();
    }

    @Override
    public void Log() {
        double value = m_valueSupplier.getAsDouble();
       
        if (m_logToNetworkTables) {
            m_publisher.set(value);
        }

        m_wpiLog.appendDouble(m_wpiEntry, value, 0);
    }
}
