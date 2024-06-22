package frc.robot.util.periodic;

import java.util.HashSet;
import java.util.Set;

/**
 * Utility class that adds support for objects with a periodic loop
 */
public final class PeriodicUtil {
    public static abstract class Periodic {
        public Periodic() {
            PeriodicUtil.add(this::periodic);
        }

        public abstract void periodic();
    }

    private static Set<Runnable> s_periodicMethods = new HashSet<Runnable>();

    public static void add(Runnable runnable) {
        s_periodicMethods.add(runnable);
    }

    public static void periodic() {
        s_periodicMethods.forEach((Runnable runnable) -> runnable.run());
    }
}
