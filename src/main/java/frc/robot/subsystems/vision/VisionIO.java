package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        double measurementTimestampSeconds = 0.0;
        Pose2d estimatedPose = new Pose2d();
    }

    public String getName();

    public boolean isSim();

    public void updateInputs(VisionIOInputs inputs);
}
