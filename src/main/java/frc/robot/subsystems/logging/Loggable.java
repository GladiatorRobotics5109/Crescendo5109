package frc.robot.subsystems.logging;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * Interface that represents a loggable value
 */
public abstract class Loggable {
    protected String m_subsystem;
    protected String m_name;

    protected boolean m_logToNetworkTables;
    
    // WPI Logger integration
    protected DataLog m_wpiLog;
    protected int m_wpiEntry;

    protected Loggable(String subsystem, String name, boolean logToNetworkTables) {
        m_subsystem = subsystem;
        m_name = name;
        m_logToNetworkTables = logToNetworkTables;

        m_wpiLog = DataLogManager.getLog();
        m_wpiEntry = m_wpiLog.start(m_subsystem, m_name);
    }

    public abstract void Log();
}
