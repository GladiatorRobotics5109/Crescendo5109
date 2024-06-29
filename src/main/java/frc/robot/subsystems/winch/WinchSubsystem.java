package frc.robot.subsystems.winch;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WinchConstants;
import frc.robot.util.Conversions;

public class WinchSubsystem extends SubsystemBase {
    private WinchIO m_io;
    private WinchIOInputsAutoLogged m_inputs;

    private boolean m_targetAngle;
    private Supplier<Rotation2d> m_desiredAngle;
    private PIDController m_anglePID;

    public WinchSubsystem() {
        switch (Constants.kCurrentMode) {
            case REAL:
                m_io = new WinchIOSparkMax(WinchConstants.kRealMotorPort);
                m_anglePID = WinchConstants.kRealAnglePID.getPIDController();

                break;
            case SIM:

                break;
            case REPLAY:
                m_io = new WinchIO() {};

                break;
        }

        m_targetAngle = false;
        m_desiredAngle = null;
    }

    public void setDesiredAngle(Supplier<Rotation2d> angleSupplier) {
        m_desiredAngle = angleSupplier;
    }

    public void setDesiredAngle(Rotation2d angle) {
        m_desiredAngle = () -> angle;
    }

    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromDegrees(
            57.8763 + (-1.07687 * Conversions.radiansToRotations(m_inputs.motorPositionRads))
        );
    }

    @Override
    public void periodic() {
        if (m_targetAngle && m_desiredAngle != null) {
            m_io.setVoltage(m_anglePID.calculate(getCurrentAngle().getRadians(), m_desiredAngle.get().getRadians()));
        }
    }
}
