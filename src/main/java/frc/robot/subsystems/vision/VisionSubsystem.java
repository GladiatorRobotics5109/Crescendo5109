package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants.VisionConstants;
import frc.robot.stateMachine.StateMachine;

public class VisionSubsystem extends SubsystemBase {
    private VisionIO[] m_visionSources;
    private VisionIOInputsAutoLogged[] m_inputs;

    private VisionSystemSim m_visionSim;

    private VisionMeasurement[] m_latestMeasurements;

    public VisionSubsystem() {
        switch (Constants.kCurrentMode) {
            case REAL:
                m_visionSources = new VisionIO[VisionConstants.kRealCameraNames.length];

                for (int i = 0; i < m_visionSources.length; i++) {
                    m_visionSources[i] = new VisionIOPhotonVision(
                        VisionConstants.kRealCameraNames[i],
                        VisionConstants.kRealRobotToCameras[i]
                    );
                }

                break;
            case REPLAY:
                // Create empty array because we don't want any IO implementations
                m_visionSources = new VisionIO[] {};

                break;
            case SIM:
                m_visionSim = new VisionSystemSim("MainVisionSystemSim");
                m_visionSim.addAprilTags(AprilTagFieldLayout.loadField(VisionConstants.kAprilTagFieldLayout));

                m_visionSources = new VisionIO[VisionConstants.kSimCameraNames.length];

                for (int i = 0; i < m_visionSources.length; i++) {
                    m_visionSources[i] = new VisionIOSim(
                        VisionConstants.kSimCameraNames[i],
                        VisionConstants.kSimRobotToCameras[i],
                        m_visionSim
                    );
                }

                break;
        }

        m_inputs = new VisionIOInputsAutoLogged[m_visionSources.length];
        for (int i = 0; i < m_visionSources.length; i++) {
            m_inputs[i] = new VisionIOInputsAutoLogged();
        }
    }

    public VisionMeasurement[] getMeasurements() {
        return m_latestMeasurements;
    }

    @Override
    public void periodic() {
        // If running vision sim, update simulator with estimated robot pose
        if (m_visionSim != null) {
            m_visionSim.update(StateMachine.SwerveState.getPose());
        }

        // Update all vision sources
        for (int i = 0; i < m_visionSources.length; i++) {
            m_visionSources[i].updateInputs(m_inputs[i]);
            Logger.processInputs("VisionInputs/" + m_visionSources[i].getName(), m_inputs[i]);
        }

        List<VisionMeasurement> measurements = new ArrayList<>();

        for (int i = 0; i < m_inputs.length; i++) {
            if (m_inputs[i].estimatedPose != null) {
                measurements.add(
                    new VisionMeasurement(
                        m_inputs[i].estimatedPose,
                        m_inputs[i].measurementTimestampSeconds,
                        m_visionSources[i].getName(),
                        m_visionSources[i].isSim()
                    )
                );
            }
        }

        m_latestMeasurements = new VisionMeasurement[measurements.size()];
        m_latestMeasurements = measurements.toArray(m_latestMeasurements);
    }
}
