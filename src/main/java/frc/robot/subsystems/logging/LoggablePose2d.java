package frc.robot.subsystems.logging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

import java.util.function.Supplier;

public class LoggablePose2d extends Loggable<Pose2d> {
    private Supplier<Pose2d> m_valueSupplier;
    private StructPublisher<Pose2d> m_publisher;
    
    public LoggablePose2d(
        String subsystem,
        String name,
        boolean logToNetworkTables,
        boolean liveLog,
        Supplier<Pose2d> valueSupplier) {
        super(subsystem, name, logToNetworkTables, liveLog);
        
        m_valueSupplier = valueSupplier;
        
        m_publisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();
    }
    
    @Override
    public void Log(Pose2d value) {
        if(m_logToNetworkTables) {
            m_publisher.set(value);
        }

        m_wpiLog.appendDoubleArray(m_wpiEntry, new double[] {value.getX(), value.getY()}, 0);
    }

    @Override
    public void Log() {
        if (m_liveLog)
            Log(m_valueSupplier.get());
    }
}