package frc.robot.subsystems.rollers.feeder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.RollersConstants.FeederConstants;
import frc.robot.util.Conversions;
import frc.robot.util.LoggedDigitalInput.LoggedDigitalInput;
import org.littletonrobotics.junction.Logger;

public class FeederSubsystem extends SubsystemBase {
    private final FeederIO m_io;
    private final FeederIOInputsAutoLogged m_inputs;

    private final PIDController m_rpmPID;
    private double m_desiredRPM;

    private final LoggedDigitalInput m_noteSensor;
    private final Trigger m_hasNoteTrigger;

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

        m_noteSensor = new LoggedDigitalInput(
            "RollersInputs/FeederInputs/NoteSensor",
            FeederConstants.kNoteSensorChannel
        );

        m_hasNoteTrigger = new Trigger(this::hasNote);
    }

    public boolean hasNote() {
        return m_inputs.motorSupplyCurrentAmps >= FeederConstants.kNoteEnterCurrentThreshold;
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

    public Trigger getHasNoteTrigger() {
        return m_hasNoteTrigger;
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
        Logger.processInputs("RollersInputs/FeederInputs", m_inputs);

        if (m_desiredRPM != 0.0) {
            m_io.setVoltage(m_rpmPID.calculate(getCurrentRPM(), m_desiredRPM));
        }
        else {
            m_rpmPID.setSetpoint(0.0);
            m_io.setVoltage(0.0);
        }
    }
}
