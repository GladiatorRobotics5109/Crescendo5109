package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.stateMachine.StateMachine;
import frc.robot.util.Conversions;
import frc.robot.util.Util;

public class ShooterSubsystem extends SubsystemBase {
    private ShooterIO m_io;
    private ShooterIOInputsAutoLogged m_inputs;

    private PIDController m_leftPID;
    private PIDController m_rightPID;

    private double m_leftDesiredRPM;
    private double m_rightDesiredRPM;

    private boolean m_autoSpinUpEnabled;

    public ShooterSubsystem() {
        m_inputs = new ShooterIOInputsAutoLogged();

        m_io = new ShooterIOSim();

        m_leftPID = ShooterConstants.kRealLeftPID.getPIDController();
        m_rightPID = ShooterConstants.kRealRightPID.getPIDController();

        m_leftDesiredRPM = 0.0;
        m_rightDesiredRPM = 0.0;

        m_autoSpinUpEnabled = true;
    }

    public double getLeftCurrentRPM() {
        return Conversions.shooterRadiansPerSecondToShooterRotationsPerMinute(m_inputs.leftMotorVelocityRadPerSec);
    }

    public double getRightCurrentRPM() {
        return Conversions.shooterRadiansPerSecondToShooterRotationsPerMinute(m_inputs.rightMotorVelocityRadPerSec);
    }

    public double getLeftDesiredRPM() {
        return m_leftDesiredRPM;
    }

    public double getRightDesiredRPM() {
        return m_rightDesiredRPM;
    }

    public void setDesiredRPM(double leftRPM, double rightRPM) {
        m_leftDesiredRPM = leftRPM;
        m_rightDesiredRPM = rightRPM;
    }

    public void setAutoSpinUpEnabled(boolean enabled) {
        m_autoSpinUpEnabled = enabled;
    }

    public boolean getAutoSpinUpEnabled() {
        return m_autoSpinUpEnabled;
    }

    public boolean isAtDesiredRPM() {
        return m_leftDesiredRPM != 0.0 && m_rightDesiredRPM != 0.0 && m_leftPID.atSetpoint() && m_rightPID.atSetpoint();
    }

    public boolean isSpinning() {
        return !MathUtil.isNear(0.0, getLeftCurrentRPM(), 1.0) || !MathUtil.isNear(0.0, getRightCurrentRPM(), 1.0);
    }

    public boolean shouldAutoSpinUp() {
        if (!m_autoSpinUpEnabled)
            return false;

        Pose2d speakerPose = Util.getTargetSpeakerPose();
        Pose2d botPose = StateMachine.SwerveState.getPose();

        if (
            botPose.getTranslation().getDistance(
                speakerPose.getTranslation()
            ) <= ShooterConstants.kAutoSpinUpRadiusMeters
        ) {
            return true;
        }

        return false;
    }

    public boolean hasDesiredRPM() {
        return m_leftDesiredRPM != 0.0 || m_rightDesiredRPM != 0.0;
    }

    public void stop() {
        setDesiredRPM(0, 0);
    }

    public void start() {
        setDesiredRPM(ShooterConstants.kShootRPM, ShooterConstants.kShootRPM);
    }

    public Command commandManualStop() {
        return this.runOnce(() -> {
            setAutoSpinUpEnabled(false);
            stop();
        });
    }

    public Command commandManualStart() {
        return this.runOnce(() -> {
            setAutoSpinUpEnabled(false);
            start();
        });
    }

    public Command commandStart() {
        return this.runOnce(this::start);
    }

    public Command commandStop() {
        return this.runOnce(this::stop);
    }

    public Command commandSetAutoSpinUpEnabled(BooleanSupplier enabled) {
        return this.runOnce(() -> setAutoSpinUpEnabled(enabled.getAsBoolean()));
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs("Shooter", m_inputs);

        // if should auto spin up and desired rpm is lower than auto spin up rpm (to allow high shoot rpm than auto spin
        // rpm)
        if (
            m_autoSpinUpEnabled && shouldAutoSpinUp()
                && (m_leftDesiredRPM <= ShooterConstants.kAutoSpinRPM
                    || m_rightDesiredRPM <= ShooterConstants.kAutoSpinRPM)
        ) {
            setDesiredRPM(ShooterConstants.kAutoSpinRPM, ShooterConstants.kAutoSpinRPM);
        }
        else if (
            m_autoSpinUpEnabled &&
                m_leftDesiredRPM == ShooterConstants.kAutoSpinRPM && m_rightDesiredRPM == ShooterConstants.kAutoSpinRPM
        ) {
            stop();
        }

        if (DriverStation.isEnabled()) {
            if (m_leftDesiredRPM == 0.0 && m_rightDesiredRPM == 0.0) {
                m_io.setVoltage(0.0, 0.0);
            }
            else {
                m_io.setVoltage(
                    m_leftPID.calculate(
                        Conversions.shooterRadiansPerSecondToShooterRotationsPerMinute(
                            m_inputs.leftMotorVelocityRadPerSec
                        ),
                        m_leftDesiredRPM
                    ),
                    m_rightPID.calculate(
                        Conversions.shooterRadiansPerSecondToShooterRotationsPerMinute(
                            m_inputs.rightMotorVelocityRadPerSec
                        ),
                        m_rightDesiredRPM
                    )
                );
            }
        }
        else {
            stop();
        }
    }
}
