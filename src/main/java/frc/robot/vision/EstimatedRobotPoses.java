package frc.robot.vision;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Wrapper for {@link EstimatedRobotPose} that implements support for multiple  {@link EstimatedRobotPose}s  and, therefore, multiple cameras!
 */
public class EstimatedRobotPoses {
    private EstimatedRobotPose[] m_estimatedRobotPoses;

    public EstimatedRobotPoses(EstimatedRobotPose ... estimatedRobotPoses) {
        // trim array to remove any null values
        int len;

        for (len = 0; len < estimatedRobotPoses.length; len++) {
            if (estimatedRobotPoses[len] == null) break;
        }
        
        m_estimatedRobotPoses = new EstimatedRobotPose[len];
        System.arraycopy(estimatedRobotPoses, 0, m_estimatedRobotPoses, 0, len);
    }

    public EstimatedRobotPose[] getEstimatedRobotPoses() {
        return m_estimatedRobotPoses;
    }

    /**
     * @return {@link Pose2d} of averaged position measurements
     */
    public Pose2d getPose2d() {
        double averageX = 0.0;
        double averageY = 0.0;
        double averageRot = 0.0;

        for (EstimatedRobotPose pose : m_estimatedRobotPoses) {
            averageX += pose.estimatedPose.getX();
            averageY += pose.estimatedPose.getY();
            averageRot += pose.estimatedPose.toPose2d().getRotation().getRadians();
        }

        return new Pose2d(
            new Translation2d(
                averageX / m_estimatedRobotPoses.length,
                averageY / m_estimatedRobotPoses.length
            ),
            Rotation2d.fromRadians(averageRot / m_estimatedRobotPoses.length)
        );
    }
}
