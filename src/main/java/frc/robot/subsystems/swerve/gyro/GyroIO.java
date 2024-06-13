package frc.robot.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean isConnected = false;
        public boolean isSim = false;
        public Rotation2d yaw = new Rotation2d();
        public Rotation2d pitch = new Rotation2d();
        public Rotation2d roll = new Rotation2d();
    }

    public default void updateInputs(GyroIOInputs inputs) {};
}
