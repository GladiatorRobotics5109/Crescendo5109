package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
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

    public ShooterSubsystem() {
        m_inputs = new ShooterIOInputsAutoLogged();

        m_io = new ShooterIOSim();

        m_leftPID = ShooterConstants.kRealLeftPID.getPIDController();
        m_rightPID = ShooterConstants.kRealRightPID.getPIDController();

        m_leftDesiredRPM = 0.0;
        m_rightDesiredRPM = 0.0;
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

    public void stop() {
        setDesiredRPM(0, 0);
    }

    private boolean shouldAutoSpinUp() {
        Pose2d speakerPose = Util.getTargetSpeakerPose();
        Pose2d botPose = StateMachine.SwerveState.getPose();

        if (
            botPose.getTranslation().getDistance(
                speakerPose.getTranslation()
            ) <= ShooterConstants.kAutoSpinUpRadiusMeters
        ) {
            Logger.recordOutput("ShooterState/ShouldAutoSpinUp", true);
            return true;
        }

        Logger.recordOutput("ShooterState/ShouldAutoSpinUp", false);
        return false;
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs("Shooter", m_inputs);

        // if should auto spin up and desired rpm is lower than auto spin up rpm (to allow high shoot rpm than auto spin
        // rpm)
        if (
            shouldAutoSpinUp() && (m_leftDesiredRPM <= ShooterConstants.kAutoSpinRPM
                || m_rightDesiredRPM <= ShooterConstants.kAutoSpinRPM)
        ) {
            setDesiredRPM(ShooterConstants.kAutoSpinRPM, ShooterConstants.kAutoSpinRPM);
        }
        else if (
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

        Logger.recordOutput(
            "ShooterState/leftCurrentRPM",
            Conversions.shooterRadiansPerSecondToShooterRotationsPerMinute(m_inputs.leftMotorVelocityRadPerSec)
        );
        Logger.recordOutput("ShooterState/leftDesiredRPM", m_leftDesiredRPM);
        Logger.recordOutput(
            "ShooterState/rightCurrentRPM",
            Conversions.shooterRadiansPerSecondToShooterRotationsPerMinute(m_inputs.rightMotorVelocityRadPerSec)
        );
        Logger.recordOutput("ShooterState/rightDesiredRPM", m_rightDesiredRPM);
    }
}
