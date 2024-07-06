package frc.robot.subsystems.rollers.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants;
import frc.robot.Constants.RollersConstants.IntakeConstants;
import frc.robot.util.Conversions;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO m_io;
    private final IntakeIOInputsAutoLogged m_inputs;

    private final PIDController m_rpmPID;
    private double m_desiredRPM;

    public IntakeSubsystem() {
        switch (Constants.kCurrentMode) {
            case REAL:
                m_io = new IntakeIOSparkMax(IntakeConstants.kRealMotorPort);
                m_rpmPID = IntakeConstants.kRealRPMPID.getPIDController();

                break;
            case SIM:
                m_io = new IntakeIOSim();
                m_rpmPID = IntakeConstants.kRealRPMPID.getPIDController();

                break;
            default:
                m_io = new IntakeIO() {};
                m_rpmPID = IntakeConstants.kRealRPMPID.getPIDController();

                break;
        }

        m_inputs = new IntakeIOInputsAutoLogged();
    }

    public void start() {
        setDesiredRPM(IntakeConstants.kIntakeRPM);
    }

    public void stop() {
        m_io.stop();
    }

    public void setDesiredRPM(double desiredRPM) {
        m_desiredRPM = desiredRPM;
    }

    public double getDesiredRPM() {
        return m_desiredRPM;
    }

    public double getCurrentRPM() {
        return Conversions.radiansPerSecondToRotationsPerMinute(m_inputs.intakeVelocityRadPerSec);
    }

    public boolean isIntaking() {
        return m_desiredRPM != 0.0;
    }

    public boolean hasNote() {
        return m_inputs.motorSupplyCurrentAmps >= IntakeConstants.kNoteEnterCurrentThreashold;
    }

    public Command commandStart() {
        return this.runOnce(this::start);
    }

    public Command commandStop() {
        return this.runOnce(this::stop).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public Command commandToggleIntake() {
        return this.runOnce(() -> {
            if (isIntaking()) {
                stop();
            }
            else {
                start();
            }
        });
    }

    public Command commandUntilNoteEnter() {
        return Commands.waitUntil(this::hasNote);
    }

    public Command commandUntilNoteExit() {
        return Commands.waitUntil(() -> !hasNote());
    }

    public Command commandWaitUntilNoteEnterExit() {
        return Commands.sequence(commandUntilNoteEnter(), commandUntilNoteExit());
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs("RollersInputs/IntakeInputs", m_inputs);

        if (m_desiredRPM != 0.0) {
            m_io.setVoltage(m_rpmPID.calculate(getCurrentRPM(), m_desiredRPM));
        }
        else {
            m_io.stop();
        }
    }
}
