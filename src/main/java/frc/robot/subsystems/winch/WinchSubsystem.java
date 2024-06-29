package frc.robot.subsystems.winch;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WinchConstants;
import frc.robot.util.Conversions;

public class WinchSubsystem extends SubsystemBase {
    private WinchIO m_io;
    private WinchIOInputsAutoLogged m_inputs;

    private boolean m_targetAngleEnabled;
    private Supplier<Rotation2d> m_targetAngleSupplier;
    private PIDController m_anglePID;

    public WinchSubsystem() {
        switch (Constants.kCurrentMode) {
            case REAL:
                m_io = new WinchIOSparkMax(WinchConstants.kRealMotorPort);
                m_anglePID = WinchConstants.kRealAnglePID.getPIDController();

                break;
            case SIM:
                m_io = new WinchIOSim();
                m_anglePID = WinchConstants.kSimAnglePID.getPIDController();

                break;
            case REPLAY:
                m_io = new WinchIO() {};
                m_anglePID = WinchConstants.kRealAnglePID.getPIDController();

                break;
        }

        m_inputs = new WinchIOInputsAutoLogged();

        m_targetAngleEnabled = false;
        m_targetAngleSupplier = null;

    }

    public Rotation2d getCurrentAngle() {
        // TODO: add better impl for encoder rot -> shooter angle
        return Rotation2d.fromDegrees(
            57.8763 + (-1.07687 * Conversions.radiansToRotations(m_inputs.motorPositionRad))
        );
    }

    public void setDesiredAngle(Supplier<Rotation2d> angleSupplier) {
        m_targetAngleSupplier = angleSupplier;
    }

    public void setDesiredAngle(Rotation2d angle) {
        m_targetAngleSupplier = () -> angle;
    }

    public void setTargetAngleEnabled(boolean enabled, Supplier<Rotation2d> angleSupplier) {
        setDesiredAngle(angleSupplier);
        setTargetAngleEnabled(enabled);
    }

    public void setTargetAngleEnabled(boolean enabled) {
        m_targetAngleEnabled = enabled;
    }

    public Command commandSetTargetAngleEnabled(BooleanSupplier enabled, Supplier<Rotation2d> angleSupplier) {
        return this.runOnce(() -> setTargetAngleEnabled(enabled.getAsBoolean(), angleSupplier));
    }

    public Command commandSetTargetAngleEnabled(BooleanSupplier enabled) {
        return this.runOnce(() -> setTargetAngleEnabled(enabled.getAsBoolean()));
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs("Winch", m_inputs);

        if (m_targetAngleEnabled && m_targetAngleSupplier != null) {
            m_io.setVoltage(
                m_anglePID.calculate(getCurrentAngle().getRadians(), m_targetAngleSupplier.get().getRadians())
            );
        }

        Logger.recordOutput("WinchState/CurrentAngle", getCurrentAngle());
    }
}
