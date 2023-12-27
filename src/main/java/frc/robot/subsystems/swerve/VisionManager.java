package frc.robot.subsystems.swerve;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Common;

/**
 * VisionManager class to get vision data from PhotonVision coprocessor
 */
public class VisionManager {
    private static boolean m_hasInit = false;

    private static PhotonCamera m_camera;
    private static PhotonPoseEstimator m_poseEstimator;

    /**
     * init vision system. SHOULD BE CALLED BEFORE VisionManager.getPose2d() is called.
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
            new Transform3d() // TODO: set correct robot to camera transform
        );


        m_hasInit = true;
    }
    
    /**
     * Get the estimated position from vision system
     */
    public static Optional<EstimatedRobotPose> getPose() {
        if (!m_hasInit) {
            DriverStation.reportWarning("VisionManager.getPose(). VisionManager has been asked for a pose, but it has not been initialized!", null);

            return null;
        }

        return m_poseEstimator.update();
    }

    public static Pose2d getPose2d() {
        Optional<EstimatedRobotPose> pose = getPose();
        
        if (pose.isEmpty()) return null;

        return pose.get().estimatedPose.toPose2d();
    }
}
