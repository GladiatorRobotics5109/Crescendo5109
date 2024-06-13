package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.struct.StructSerializable;

public class VisionMeasurement implements StructSerializable {
    public static final VisionMeasurementStruct struct = new VisionMeasurementStruct();

    private Pose2d m_estimatedPose;
    private double m_timestamp;
    private String m_cameraName;

    /**
     * Was this measurement from a simulated camera
     */
    private boolean m_isFromSimCamera;

    public VisionMeasurement(Pose2d estimatedPose, double timestamp, String cameraName, boolean isFromSimCamera) {
        m_estimatedPose = estimatedPose;
        m_timestamp = timestamp;
        m_cameraName = cameraName;
        m_isFromSimCamera = isFromSimCamera;
    }

    public Pose2d getEstimatedPose() {
        return m_estimatedPose;
    }

    public double getTimestamp() {
        return m_timestamp;
    }

    public String getCameraName() {
        return m_cameraName;
    }

    public boolean isFromSimCamera() {
        return m_isFromSimCamera;
    }
}
