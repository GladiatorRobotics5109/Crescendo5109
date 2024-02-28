package frc.robot.util.logging;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class Logger {
    private static Logger s_instance;

    private final ArrayList<Loggable<?>> m_loggables;

    private Logger(Loggable<?>... loggables) {
        DataLogManager.start();

        m_loggables = new ArrayList<Loggable<?>>(Arrays.asList(loggables));
    }

    public static void init(Loggable<?> ... loggables) {
        s_instance = new Logger(loggables);
    }

    public static Logger getInstance() {
        return s_instance;
    }

    public static void addLoggable(Loggable<?> loggable) {
        s_instance.addLoggableImpl(loggable);
    }

    public static void info(String message) {
        s_instance.infoImpl(message);
    }

    public static void warn(String messag) {
        s_instance.warnImpl(messag);
    }

    public static void error(String message, StackTraceElement[] stackTrace) {
        s_instance.errorImpl(message, stackTrace);
    }

    private void addLoggableImpl(Loggable<?> loggable) {
        m_loggables.add(loggable);
    }

    private void infoImpl(String message) {
        DataLogManager.log("[INFO]: " + message);
    }

    private void warnImpl(String message) {
        DriverStation.reportWarning(message, new StackTraceElement[] {});
        DataLogManager.log("[WARN]: " + message);
    }

    private void errorImpl(String message, StackTraceElement[] stackTrace) {
        DriverStation.reportError(message, stackTrace);
        DataLogManager.log("[ERR]: " + message);
    }

    public void periodic() {
        updateLog();
    }

    private void updateLog() {
        for (Loggable<?> loggable : m_loggables) {
            loggable.log();
        }
    }
}
