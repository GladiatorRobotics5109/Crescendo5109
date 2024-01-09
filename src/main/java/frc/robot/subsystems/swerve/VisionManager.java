package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Common;
import frc.robot.Util.EstimatedPose;

/**
 * VisionManager class to get vision data from PhotonVision coprocessor
 */
public class VisionManager {
    private static boolean m_hasInit = false;

    private static PhotonCamera m_camera;
    private static PhotonPoseEstimator m_poseEstimator;

    private static List<EstimatedPose> m_poses;

    /**
     * init vision system. SHOULD BE CALLED FIRST.
     */
    public static void init() {
        if (Common.currentAprilTagFieldLayout == null) {
            DriverStation.reportError("VisionManager failed to initialize! Common.currentAprilTagFieldLayout == null", null);

            return;
        }

        // TODO: set correct camera name
        m_camera = new PhotonCamera("testCam");

        m_poseEstimator = new PhotonPoseEstimator(
            Common.currentAprilTagFieldLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            m_camera,
            new Transform3d() // TODO: set correct robot -> camera transform
        );

        m_poses = new ArrayList<EstimatedPose>();

        m_hasInit = true;
    }
    
    /**
     * Get the estimated position from vision system
     */
    public static Optional<EstimatedPose> getPose() {
        if (!m_hasInit) {
            DriverStation.reportWarning("VisionManager.getPose(). VisionManager has been asked for a pose, but it has not been initialized!", null);

            return Optional.empty();
        }

        Optional<EstimatedRobotPose> estimatedPose = m_poseEstimator.update();

        return Optional.of(
            new EstimatedPose(
                estimatedPose.get(),
                EstimatedPose.getDeviationOf(m_poses)
            )
        );
    }

    /** Clear poses used for confidence */
    public static void clearCachedPoses() {
        m_poses.clear();
    }
}
