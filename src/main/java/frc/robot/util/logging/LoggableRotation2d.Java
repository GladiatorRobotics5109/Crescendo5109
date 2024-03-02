package frc.robot.util.logging;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.StructLogEntry;

import java.util.function.Supplier;

public final class LoggableRotation2d extends Loggable<Rotation2d> {
    private Supplier<Rotation2d> m_valueSupplier;
    private StructPublisher<Rotation2d> m_publisher;
    private StructLogEntry<Rotation2d> m_wpiEntry;

    public LoggableRotation2d(String name, boolean logToNetworkTables, boolean liveLog, Supplier<Rotation2d> valueSupplier) {
        super(name, logToNetworkTables, liveLog);

        m_valueSupplier = valueSupplier;

        m_publisher = NetworkTableInstance.getDefault().getStructTopic(name, Rotation2d.struct).publish();

        m_wpiEntry = StructLogEntry.create(m_wpiLog, name, Rotation2d.struct);
    }

    public LoggableRotation2d(String name, boolean logToNetworkTables) {
        super(name, logToNetworkTables);

        m_publisher = NetworkTableInstance.getDefault().getStructTopic(name, Rotation2d.struct).publish();

        m_wpiEntry = StructLogEntry.create(m_wpiLog, name, Rotation2d.struct);
    }

    public LoggableRotation2d(String name) {
        super(name);
        
        m_publisher = NetworkTableInstance.getDefault().getStructTopic(name, Rotation2d.struct).publish();

        m_wpiEntry = StructLogEntry.create(m_wpiLog, name, Rotation2d.struct);
    }

    @Override
    public void log(Rotation2d value) {
        if (m_logToNetworkTables) {
            m_publisher.set(value);
        }

        m_wpiEntry.append(value);
    }
    
    @Override
    protected void log() {
        if(m_liveLog)
            log(m_valueSupplier.get());
    }
}