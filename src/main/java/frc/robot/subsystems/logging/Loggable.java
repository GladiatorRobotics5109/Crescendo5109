package frc.robot.subsystems.logging;

public abstract class Loggable {
    private String m_subsystem;
    private String m_name;
    private boolean m_logToNetworkTables;

    public Loggable(String subsystem, String name, boolean logToNetworkTables) {
        m_subsystem = subsystem;
        m_name = name;
        m_logToNetworkTables = logToNetworkTables;
    }

    public abstract void Log();
}
