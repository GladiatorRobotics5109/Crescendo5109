package frc.robot.util.logging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;

import java.util.function.Supplier;

/**
 * @apiNote {@link LoggablePose2d} logs to the wpi logger as a string (x, y, rot [int rad])
 */
public final class LoggablePose2d extends Loggable<Pose2d> {
    private Supplier<Pose2d> m_valueSupplier;
    private StructPublisher<Pose2d> m_publisher;
    private StringLogEntry m_wpiEntry;
    
    public LoggablePose2d(
        String name,
        boolean logToNetworkTables,
        boolean liveLog,
        Supplier<Pose2d> valueSupplier) {
        super(name, logToNetworkTables, liveLog);
        
        m_valueSupplier = valueSupplier;
        
        m_publisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();

        m_wpiEntry = new StringLogEntry(m_wpiLog, name);
    }

    public LoggablePose2d(String name, boolean logToNetworkTables) {
        super(name, logToNetworkTables);

        m_publisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();

        m_wpiEntry = new StringLogEntry(m_wpiLog, name);
    }

    public LoggablePose2d(String name) {
        super(name);

        m_publisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();

        m_wpiEntry = new StringLogEntry(m_wpiLog, name);
    }
    
    @Override
    public void log(Pose2d value) {
        if(m_logToNetworkTables) {
            m_publisher.set(value);
        }

        m_wpiEntry.append("(" + value.getX() + ", " + value.getY() + ", " + value.getRotation().getRadians() + "rad)");
    }

    @Override
    protected void log() {
        if (m_liveLog)
            log(m_valueSupplier.get());
    }
}