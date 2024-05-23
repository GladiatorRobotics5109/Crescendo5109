package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean isConnected = false;
        public Rotation2d yaw;
        public Rotation2d pitch;
        public Rotation2d roll;
    }

    public default void updateInputs(GyroIOInputs inputs) {
    };
}
