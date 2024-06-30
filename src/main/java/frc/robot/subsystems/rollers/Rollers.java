package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.rollers.feeder.FeederSubsystem;
import frc.robot.subsystems.rollers.intake.IntakeSubsystem;

public class Rollers {
    private final IntakeSubsystem m_intake;
    private final FeederSubsystem m_feeder;

    public Rollers() {
        m_intake = new IntakeSubsystem();
        m_feeder = new FeederSubsystem();
    }

    public double getIntakeDesiredRPM() {
        return m_intake.getDesiredRPM();
    }

    public double getIntakeCurrentRPM() {
        return m_intake.getCurrentRPM();
    }

    public boolean getIntakeIsIntaking() {
        return m_intake.isIntaking();
    }

    public boolean getIntakeHasNote() {
        return m_intake.hasNote();
    }

    public boolean getFeederHasNote() {
        return m_feeder.hasNote();
    }

    public double getFeederDesriedRPM() {
        return m_feeder.getDesiredRPM();
    }

    public double getFeederCurrentRPM() {
        return m_feeder.getCurrentRPM();
    }

    public boolean hasNote() {
        return m_intake.hasNote() || m_feeder.hasNote();
    }

    public Command commandStartIntakeProcedure() {
        return Commands.sequence(
            m_intake.commandStart(),
            m_feeder.commandStart(),
            m_intake.commandWaitUntilNoteEnterExit(),
            m_intake.commandStop(),
            m_feeder.commandStop()
        ).withInterruptBehavior(InterruptionBehavior.kCancelSelf).handleInterrupt(
            () -> DriverStation.reportWarning(
                "CommandBuilder.commandStartIntakeProcedure() command has been interrupted!",
                false
            )
        );
    }
}
