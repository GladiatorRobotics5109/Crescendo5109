package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.BangBangController;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.stateMachine.StateMachine;
import frc.robot.util.Conversions;
import frc.robot.util.Util;

public class ShooterSubsystem extends SubsystemBase {
    private ShooterIO m_io;
    private ShooterIOInputsAutoLogged m_inputs;

    private BangBangController m_leftBangBang;
    private BangBangController m_rightBangBang;

    private double m_leftDesiredRPM;
    private double m_rightDesiredRPM;

    private boolean m_autoSpinUpEnabled;
    /* Wether or not the current desired rpm is because of auto spin up algorithm or not */
    private boolean m_isAutoSpinningUp;

    public ShooterSubsystem() {
        m_inputs = new ShooterIOInputsAutoLogged();

        switch (Constants.kCurrentMode) {
            case REAL:
                m_io = new ShooterIOSparkMax(ShooterConstants.kLeftMotorPort, ShooterConstants.kRightMotorPort);

                break;
            case SIM:
                m_io = new ShooterIOSim();

                break;
            default:
                m_io = new ShooterIO() {};

                break;
        }

        m_leftBangBang = new BangBangController(ShooterConstants.kRPMTolerance);
        m_rightBangBang = new BangBangController(ShooterConstants.kRPMTolerance);

        m_leftDesiredRPM = 0.0;
        m_rightDesiredRPM = 0.0;

        m_autoSpinUpEnabled = true;
        m_isAutoSpinningUp = false;
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
        return m_leftDesiredRPM != 0.0 && m_rightDesiredRPM != 0.0 && m_leftBangBang.atSetpoint()
            && m_rightBangBang.atSetpoint();
    }

    public boolean isSpinning() {
        return !MathUtil.isNear(0.0, getLeftCurrentRPM(), 1.0) || !MathUtil.isNear(0.0, getRightCurrentRPM(), 1.0);
    }

    public boolean shouldAutoSpinUp() {
        if (!m_autoSpinUpEnabled)
            return false;

        Pose2d speakerPose = Util.getTargetSpeakerPose();
        Pose2d botPose = StateMachine.SwerveState.getPose();

        return botPose.getTranslation().getDistance(
            speakerPose.getTranslation()
        ) <= ShooterConstants.kAutoSpinUpRadiusMeters;
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

    public Command commandSetAutoSpinUpEnabled(boolean enabled) {
        return this.runOnce(() -> setAutoSpinUpEnabled(enabled));
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs("ShooterInputs", m_inputs);

        // if should auto spin up and desired rpm is lower than auto spin up rpm (to allow higher shoot rpm than auto
        // spin
        // rpm)
        if (
            shouldAutoSpinUp() && (m_leftDesiredRPM <= ShooterConstants.kAutoSpinRPM
                || m_rightDesiredRPM <= ShooterConstants.kAutoSpinRPM)
        ) {
            setDesiredRPM(ShooterConstants.kAutoSpinRPM, ShooterConstants.kAutoSpinRPM);
            m_isAutoSpinningUp = true;
        }
        else if (!shouldAutoSpinUp() && m_isAutoSpinningUp) {
            stop();
            m_isAutoSpinningUp = false;
        }

        if (m_leftDesiredRPM == 0.0 && m_rightDesiredRPM == 0.0) {
            m_io.setVoltage(0.0, 0.0);
        }

        if (DriverStation.isEnabled()) {
            m_io.setVoltage(
                m_leftBangBang.calculate(
                    Conversions.shooterRadiansPerSecondToShooterRotationsPerMinute(
                        m_inputs.leftMotorVelocityRadPerSec
                    ),
                    m_leftDesiredRPM
                ) * 12,
                m_rightBangBang.calculate(
                    Conversions.shooterRadiansPerSecondToShooterRotationsPerMinute(
                        m_inputs.rightMotorVelocityRadPerSec
                    ),
                    m_rightDesiredRPM
                ) * 12
            );
        }
        else {
            stop();
        }
    }
}
