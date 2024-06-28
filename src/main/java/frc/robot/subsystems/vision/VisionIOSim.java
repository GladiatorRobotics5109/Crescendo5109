package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.SwerveConstants.VisionConstants;
import frc.robot.Constants.SwerveConstants.VisionConstants.SimCameraConstants;

public class VisionIOSim implements VisionIO {
    private PhotonCameraSim m_camera;
    private PhotonPoseEstimator m_poseEstimator;

    public VisionIOSim(String cameraName, Transform3d robotToCamera, VisionSystemSim sim) {
        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setAvgLatencyMs(SimCameraConstants.kAverageLatencyMs);
        cameraProp.setFPS(SimCameraConstants.kFPS);
        cameraProp.setCalibration(
            SimCameraConstants.kImageWidth,
            SimCameraConstants.kImageHeight,
            SimCameraConstants.kFov
        );
        cameraProp.setLatencyStdDevMs(SimCameraConstants.kLatencyStdDevMs);

        m_camera = new PhotonCameraSim(new PhotonCamera(cameraName), cameraProp, 0.01, 5.0);
        m_camera.enableDrawWireframe(true);
        m_camera.enableProcessedStream(true);
        m_camera.enableRawStream(true);
        // CameraServer.addCamera(m_camera.getVideoSimRaw());

        m_poseEstimator = new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(VisionConstants.kAprilTagFieldLayout),
            VisionConstants.kPoseEstimationStrategy,
            m_camera.getCamera(),
            robotToCamera
        );

        sim.addCamera(m_camera, robotToCamera);
    }

    @Override
    public String getName() {
        return m_camera.getCamera().getName();
    }

    @Override
    public boolean isSim() {
        return true;
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
