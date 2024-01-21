package frc.robot.vision;

import java.util.Optional;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.io.IOException;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;


public class VisionManager {

    private final List<PhotonCamera> m_cameras= new ArrayList<>();
    private final List<PhotonPoseEstimator> m_estimators= new ArrayList<>();
    private AprilTagFieldLayout m_aprilTagFieldLayout;
    private PhotonCamera m_camera;
    private PhotonPoseEstimator m_estimator;



    public VisionManager(Map<String, Transform3d> visionSources) {
        
        try {
            m_aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(Constants.VisionConstants.kApriltagLayout.m_resourceFile);
        } catch (IOException e) {
            DriverStation.reportError("Failed to load field layout!", e.getStackTrace());
            System.err.println("Failed to load field layout!");
            e.printStackTrace();  
        }

        m_camera = new PhotonCamera("CameraOne");
        
        m_estimator = new PhotonPoseEstimator(
            m_aprilTagFieldLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            m_camera, 
            Constants.VisionConstants.kCameraPos
        );

        // for (Map.Entry<String, Transform3d> source : visionSources.entrySet()) {
        //     PhotonCamera camera = new PhotonCamera(source.getKey());

        //     PhotonPoseEstimator visionEstimator = new PhotonPoseEstimator(
        //         m_aprilTagFieldLayout,
        //         PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        //         camera, 
        //         source.getValue()
        //     );
        //     m_cameras.add(camera);
        //     m_estimators.add(visionEstimator);
        // }
        
    }

    public Optional<EstimatedVisionPosition> getPose() {
        List<EstimatedRobotPose> estimatedPoses = new ArrayList<EstimatedRobotPose>();

        // for (PhotonPoseEstimator estimator : m_estimators) {

        //     Optional<EstimatedRobotPose> estimatedPose = estimator.update();
            
        //     if (estimatedPose.isEmpty()) {
        //         System.out.println("Empty!");
        //         continue; 
        //     }

        //     System.out.println("Vision: (" + estimatedPose.get().estimatedPose.getX() + ", " + estimatedPose.get().estimatedPose.getY() + ")");

        //     estimatedPoses.add(estimatedPose.get());
        // }
        Optional<EstimatedRobotPose> estimatedPose = m_estimator.update();

        if (estimatedPose.isEmpty()) {
            System.out.println("Empty!");
            
            return Optional.empty();
        }

        System.out.println("Vision: (" + estimatedPose.get().estimatedPose.getX() + ", " + estimatedPose.get().estimatedPose.getY() + ")");

        return Optional.of(new EstimatedVisionPosition(estimatedPose.get()));
    }
}
