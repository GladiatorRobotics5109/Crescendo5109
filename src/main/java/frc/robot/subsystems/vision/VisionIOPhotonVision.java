package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class VisionIOPhotonVision implements VisionIO {
    private final PhotonCamera m_camera;
    private final PhotonPoseEstimator m_poseEstimator;
    private AprilTagFieldLayout m_fieldLayout;

    public VisionIOPhotonVision(
        String cameraName, Transform3d robotToCamera
    ) {
        try {
            m_fieldLayout = AprilTagFieldLayout
                .loadFromResource(Constants.SwerveConstants.VisionConstants.kAprilTagFieldLayout.m_resourceFile);
        }
        catch (IOException e) {
            DriverStation.reportError("Failed to load apriltag field layout!", e.getStackTrace());
            m_fieldLayout = null;
        }

        m_camera = new PhotonCamera(cameraName);

        m_poseEstimator = new PhotonPoseEstimator(
            m_fieldLayout,
            Constants.SwerveConstants.VisionConstants.kPoseEstimationStrategy,
            m_camera,
            robotToCamera
        );
    }

    @Override
    public String getName() {
        return m_camera.getName();
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        Optional<EstimatedRobotPose> estimatedPose = m_poseEstimator.update();
        if (estimatedPose.isPresent()) {
            inputs.estimatedPose = estimatedPose.get().estimatedPose.toPose2d();
            inputs.measurementTimestampSeconds = estimatedPose.get().timestampSeconds;
        }
        else {
            inputs.estimatedPose = null;
            inputs.measurementTimestampSeconds = Double.NaN;
        }
    }
}
