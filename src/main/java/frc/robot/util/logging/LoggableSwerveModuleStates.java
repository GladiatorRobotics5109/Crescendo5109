package frc.robot.util.logging;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.datalog.StringLogEntry;

import java.util.function.Supplier;

/**
 * @apiNote {@link LoggableSwerveModuleStates} logs to the wpi logger as a string.
 */
public final class LoggableSwerveModuleStates extends Loggable<SwerveModuleState[]> {
    private Supplier<SwerveModuleState[]> m_valueSupplier;
    
    private StructArrayPublisher<SwerveModuleState> m_publisher;
    private StringLogEntry m_wpiEntry;
    
    public LoggableSwerveModuleStates(
        String name,
        boolean logToNetworkTables,
        boolean liveLog,
        Supplier<SwerveModuleState[]> valueSupplier) {
        super(name, logToNetworkTables, liveLog);
        
        m_valueSupplier = valueSupplier;
        
        m_publisher = NetworkTableInstance.getDefault().getStructArrayTopic(name, SwerveModuleState.struct).publish();
        
        m_wpiEntry = new StringLogEntry(m_wpiLog, name);
    }
    
    public LoggableSwerveModuleStates(String name, boolean logToNetworkTables) {
        super(name, logToNetworkTables);
        
        m_publisher = NetworkTableInstance.getDefault().getStructArrayTopic(name, SwerveModuleState.struct).publish();

        m_wpiEntry = new StringLogEntry(m_wpiLog, name);
    }
    
    public LoggableSwerveModuleStates(String name) {
        super(name);
        
        m_publisher = NetworkTableInstance.getDefault().getStructArrayTopic(name, SwerveModuleState.struct).publish();

        m_wpiEntry = new StringLogEntry(m_wpiLog, name);
    }

    @Override
    public void log(SwerveModuleState[] value) {
        if (m_logToNetworkTables) {
            m_publisher.set(value);
        }
        
        StringBuilder strLog = new StringBuilder();
        for (int i = 0; i < value.length; i++) {
            strLog.append("SwerveModuleState ").append(i).append(":").append("speedMetersPerSecond: ").append(value[i].speedMetersPerSecond).append("\n").append("angle radians: ").append(value[i].angle.getRadians()).append("\n");
        }
        
        m_wpiEntry.append(strLog.toString());
    }

    @Override
    protected void log() {
        if(m_liveLog) {
            log(m_valueSupplier.get());
        }
    }
}