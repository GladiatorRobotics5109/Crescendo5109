package frc.robot.subsystems.winch;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;

import com.github.gladiatorrobotics5109.gladiatorroboticslib.advantagekitutil.loggedpidcontroller.LoggedPIDController;
import org.littletonrobotics.junction.Logger;

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
    private LoggedPIDController m_anglePID;
    private Rotation2d m_targetAngle;

    public WinchSubsystem() {
        switch (Constants.kCurrentMode) {
            case REAL:
                m_io = new WinchIOSparkMax(WinchConstants.kRealMotorPort);
                m_anglePID = WinchConstants.kRealAnglePID.getLoggedPIDController("WinchState/AnglePID");

                break;
            case SIM:
                m_io = new WinchIOSim();
                m_anglePID = WinchConstants.kSimAnglePID.getLoggedPIDController("WinchState/AnglePID");

                break;
            case REPLAY:
                m_io = new WinchIO() {};
                m_anglePID = WinchConstants.kRealAnglePID.getLoggedPIDController("WinchState/AnglePID");

                break;
        }

        m_inputs = new WinchIOInputsAutoLogged();

        m_targetAngleEnabled = false;
        m_targetAngleSupplier = null;
        m_targetAngle = WinchConstants.kStartingAngle;

        setWinchAngle(WinchConstants.kStartingAngle);
    }

    public Rotation2d getCurrentAngle() {
        return Conversions.winchMotorRadiansToWinchAngle(m_inputs.motorPositionRad);
    }

    public Rotation2d getTargetAngle() {
        return m_targetAngle;
    }

    public boolean isAtTargetAngle() {
        return m_targetAngleEnabled && m_anglePID.atSetpoint();
    }

    public boolean isTargetingAngle() {
        return m_targetAngleEnabled;
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

    private void setWinchAngle(Rotation2d angle) {
        m_io.setMotorPosition(Rotation2d.fromRadians(Conversions.winchAngleToWinchMotorRadians(angle)));
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs("WinchInputs", m_inputs);

        if (m_targetAngleEnabled && m_targetAngleSupplier != null) {
            m_targetAngle = m_targetAngleSupplier.get();
            if (m_targetAngle.getRadians() > WinchConstants.kMaxAngle.getRadians()) {
                DriverStation.reportWarning(
                    "The WinchSubsystem has been requested to reach an angle of " + m_targetAngle.getDegrees()
                        + "deg but its max angle is " + WinchConstants.kMaxAngle.getDegrees() + "deg.",
                    false
                );

                m_targetAngle = WinchConstants.kMaxAngle;
            }
            else if (m_targetAngle.getRadians() < WinchConstants.kMinAngle.getRadians()) {
                DriverStation.reportWarning(
                    "The WinchSubsystem has been requested to reach an angle of " + m_targetAngle.getDegrees()
                        + "deg but its min angle is " + WinchConstants.kMinAngle.getDegrees() + "deg.",
                    false
                );

                m_targetAngle = WinchConstants.kMinAngle;
            }

            m_io.setVoltage(
                m_anglePID.calculate(getCurrentAngle().getRadians(), m_targetAngle.getRadians())
            );
        }
        else {
            m_io.setVoltage(0);
        }

        Logger.recordOutput("WinchState/CurrentAngle", getCurrentAngle());
    }
}
