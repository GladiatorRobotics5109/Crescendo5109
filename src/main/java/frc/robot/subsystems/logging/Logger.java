package frc.robot.subsystems.logging;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Singleton (digital) subystem that manages logging to multiple mediums
 */
public class Logger extends SubsystemBase {
    private ArrayList<Loggable> m_loggables;

    private static Logger s_instance;

    private Logger(Loggable ... loggables) {
        DataLogManager.start();

        m_loggables = new ArrayList<Loggable>(Arrays.asList(loggables));
    }

    public static void init(Loggable ... loggables) {
        s_instance = new Logger(loggables);
    }

    public static Logger getInstance() {
        return s_instance;
    }

    public void addLoggable(Loggable loggable) {
        m_loggables.add(loggable);
    }

    private void updateLog() {
        for (Loggable loggable : m_loggables) {
            loggable.Log();
        }
    }

    public void info(String message) {
        DataLogManager.log("[INFO]: " + message);
    }

    public void warn(String message) {
        DataLogManager.log("[WARN]: " + message);
    }

    public void error(String message) {
        DataLogManager.log("[ERR]: " + message);
    }

    @Override
    public void periodic() {
        updateLog();
    }
}
