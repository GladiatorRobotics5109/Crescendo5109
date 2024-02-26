package frc.robot.subsystems.logging;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * Interface that represents a loggable value
 */
public abstract class Loggable<T> {
    protected String m_name;

    protected boolean m_logToNetworkTables;
    protected boolean m_liveLog;
    
    // WPI Logger integration
    protected DataLog m_wpiLog;

    protected Loggable(String name, boolean logToNetworkTables, boolean liveLog) {
        m_name = name;
        m_logToNetworkTables = logToNetworkTables;
        m_liveLog = liveLog;

        m_wpiLog = DataLogManager.getLog();
    }

    protected Loggable(String name, boolean logToNetworkTables) {
        m_name = name;
        m_logToNetworkTables = logToNetworkTables;
        m_liveLog = false;

        m_wpiLog = DataLogManager.getLog();
    }

    protected Loggable(String name) {
        m_name = name;
        m_logToNetworkTables = true;
        m_liveLog = true;
        
        m_wpiLog = DataLogManager.getLog();
    }

    /**
     * Marked as private here because without any parameters should only really be called by {@link Logger} in this same package.
     */
    protected abstract void log();

    public abstract void log(T value);
}
