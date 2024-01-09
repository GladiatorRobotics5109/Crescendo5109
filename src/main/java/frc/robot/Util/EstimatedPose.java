package frc.robot.Util;

import java.util.List;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class EstimatedPose {
    private EstimatedRobotPose m_pose;
    private Matrix<N3, N1> m_confidence;

    public EstimatedPose(EstimatedRobotPose robotPose, Matrix<N3, N1> confidence) {
        m_pose = robotPose;
        m_confidence = confidence;
    }

    public Pose2d getPose() {
        return m_pose.estimatedPose.toPose2d();
    }

    public double getX() {
        return m_pose.estimatedPose.toPose2d().getX();
    }

    public double getY() {
        return m_pose.estimatedPose.toPose2d().getY();
    }

    public Rotation2d getRot() {
        return m_pose.estimatedPose.toPose2d().getRotation();
    }

    public double getTimestampSeconds() {
        return m_pose.timestampSeconds;
    }

    public Matrix<N3, N1> getConfidence() {
        return m_confidence;
    }

    public EstimatedRobotPose getEstimatedRobotPose() {
        return m_pose;
    }

    /**
     * Utility function that computes standard deviation of a list of EstimatedPoses
     * @param poses List of EstimatedPoses
     * @return Matrix with computed standard deviations
     */
    public static Matrix<N3, N1> getDeviationOf(List<EstimatedPose> poses) {
        // stdDev = sqrt(((x-mean)^2...) / size)

        Matrix<N3, N1> means = getMeanFromEstimatedPoses(poses);

        double numX = 0.0;
        double numY = 0.0;
        double numRot = 0.0;
        
        for(EstimatedPose pose : poses) {
            numX *= Math.pow(
                (pose.getX() - means.get(0, 0)), 
                2
            );
            numY *= Math.pow(
                (pose.getY() - means.get(1, 0)), 
                2
            );
            numRot *= Math.pow(
                (pose.getRot().getRadians() - means.get(2, 0)), 
                2
            );
        }

        double stdDevX = Math.sqrt(numX / poses.size());
        double stdDevY = Math.sqrt(numY / poses.size());
        double stdDevRot = Math.sqrt(numRot / poses.size());

        Matrix<N3, N1> result = new Matrix<N3, N1>(Nat.N3(), Nat.N1());

        result.set(0, 0, stdDevX);
        result.set(0, 0, stdDevY);
        result.set(0, 0, stdDevRot);

        return result;
    }

    /**
     * 
     * @param poses List of all poses
     * @return Matrix of average poses
     */
    private static Matrix<N3, N1> getMeanFromEstimatedPoses(List<EstimatedPose> poses) {
        double totalX = 0.0;
        double totalY = 0.0;
        double totalRot = 0.0;

        for (EstimatedPose pose : poses) {
            totalX += pose.getX();
            totalY += pose.getY();
            totalRot += pose.getRot().getRadians();
        }

        Matrix<N3, N1> result = new Matrix<N3, N1>(Nat.N3(), Nat.N1());

        result.set(0, 0, totalX / poses.size());
        result.set(1, 0, totalY / poses.size());
        result.set(2, 0, totalRot / poses.size());

        return result;
    }
}
