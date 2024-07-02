package frc.robot.subsystems.rollers.feeder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants;
import frc.robot.Constants.RollersConstants.FeederConstants;
import frc.robot.util.Conversions;
import org.littletonrobotics.junction.Logger;

public class FeederSubsystem extends SubsystemBase {
    private final FeederIO m_io;
    private final FeederIOInputsAutoLogged m_inputs;

    private final PIDController m_rpmPID;
    private double m_desiredRPM;

    public FeederSubsystem() {
        switch (Constants.kCurrentMode) {
            case REAL:
                m_io = new FeederIOSparkMax(FeederConstants.kRealMotorPort);
                m_rpmPID = FeederConstants.kRealRPMPID.getPIDController();

                break;
            case SIM:
                m_io = new FeederIOSim();
                m_rpmPID = FeederConstants.kSImRPMPID.getPIDController();

                break;
            default:
                m_io = new FeederIO() {};
                m_rpmPID = FeederConstants.kRealRPMPID.getPIDController();

                break;
        }

        m_desiredRPM = 0.0;

        m_inputs = new FeederIOInputsAutoLogged();
    }

    public boolean hasNote() {
        return m_inputs.motorSupplyCurrentAmps >= FeederConstants.kNoteEnterCurrentThreashold;
    }

    public boolean isIntaking() {
        return m_desiredRPM != 0.0;
    }

    public double getDesiredRPM() {
        return m_desiredRPM;
    }

    public double getCurrentRPM() {
        return Conversions.radiansPerSecondToRotationsPerMinute(m_inputs.feederVelocityRadPerSec);
    }

    public void setDesiredRPM(double desiredRPM) {
        m_desiredRPM = desiredRPM;
    }

    public void start() {
        setDesiredRPM(FeederConstants.kFeedRPM);
    }

    public void stop() {
        setDesiredRPM(0.0);
        m_io.setVoltage(0.0);
    }

    public Command commandStart() {
        return this.runOnce(this::start);
    }

    public Command commandStop() {
        return this.runOnce(this::stop).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs("Rollers/Feeder", m_inputs);

        if (m_desiredRPM != 0.0) {
            m_io.setVoltage(m_rpmPID.calculate(getCurrentRPM(), m_desiredRPM));
        }
        else {
            m_rpmPID.setSetpoint(0.0);
            m_io.setVoltage(0.0);
        }
    }
}
