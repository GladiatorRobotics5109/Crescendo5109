package frc.robot.subsystems.swerve.swerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants.SwerveModuleConstants;
import frc.robot.util.Conversions;
import frc.robot.util.InvalidSwerveModuleMotorConfigurationException;
import frc.robot.util.MotorControllerType;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    private final SwerveModuleIO m_io;
    private final SwerveModuleIOInputsAutoLogged m_inputs;
    private final int m_index;

    private final SimpleMotorFeedforward m_driveFeedforward;
    private final PIDController m_drivePID;
    private final PIDController m_turnPID;

    private Rotation2d m_turnAngleSetpoint;
    private Double m_driveWheelSpeedSetpointRadPerSec;
    private Rotation2d m_turnRelativeOffset;
    private SwerveModulePosition[] m_odometryPositions;

    private Translation2d m_moduleTranslation2d;

    public SwerveModule(SwerveModuleConstants constants) throws InvalidSwerveModuleMotorConfigurationException {
        MotorControllerType driveController = constants.driveMotorController;
        MotorControllerType turnController = constants.turnMotorController;

        if (Constants.kCurrentMode == Constants.Mode.REPLAY) {
            m_io = new SwerveModuleIO() {};
        }
        else if (driveController == MotorControllerType.TalonFX && turnController == MotorControllerType.SparkMAX) {
            m_io = new SwerveModuleIOTalonFXDriveSparkMaxTurn(
                constants.driveMotorPort,
                constants.turnMotorPort,
                false,
                constants.turnAbsoluteEncoderOffset
            );
        }
        else if (
            driveController == MotorControllerType.Sim && turnController == MotorControllerType.Sim
                && Constants.kCurrentMode == Constants.Mode.SIM
        ) {
            m_io = new SwerveModuleIOSim();
        }
        else {
            throw new InvalidSwerveModuleMotorConfigurationException(
                "Illegal Swerve Module motor controller configuration!"
            );
        }

        m_inputs = new SwerveModuleIOInputsAutoLogged();

        m_index = constants.index;

        m_driveFeedforward = constants.driveFeedforward.get();
        m_drivePID = constants.drivePID.get();
        m_turnPID = constants.turnPID.get();
        m_turnPID.enableContinuousInput(0, 2 * Math.PI);

        m_turnAngleSetpoint = null;
        m_driveWheelSpeedSetpointRadPerSec = null;

        m_turnRelativeOffset = constants.turnRelativeOffset;

        m_odometryPositions = new SwerveModulePosition[] {};

        m_moduleTranslation2d = constants.moduleTranslation2d;
    }

    public SwerveModuleState setDesiredState(SwerveModuleState desiredState) {
        return setDesiredState(desiredState, true);
    }

    public SwerveModuleState setDesiredState(SwerveModuleState desiredState, boolean optimize) {
        SwerveModuleState state = optimize ? SwerveModuleState.optimize(desiredState, getAngle()) : desiredState;

        m_turnAngleSetpoint = state.angle;
        m_driveWheelSpeedSetpointRadPerSec = Conversions.WheelMToWheelRad(state.speedMetersPerSecond);

        return state;
    }

    public Rotation2d getAngle() {
        return m_inputs.turnPosition.plus(m_turnRelativeOffset);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Conversions.WheelRadToWheelM(m_inputs.drivePositionRad),
            m_inputs.turnPosition
        );
    }

    public Translation2d getTranslation2d() {
        return m_moduleTranslation2d;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Conversions.WheelRadToWheelM(m_inputs.driveVelocityRadPerSec),
            getAngle()
        );
    }

    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs("Swerve/Module" + m_index, m_inputs);

        if (m_turnRelativeOffset == null) {
            m_turnRelativeOffset = m_inputs.turnAbsolutePosition.minus(m_inputs.turnPosition);
        }

        if (m_turnAngleSetpoint != null) {
            m_io.setTurnVoltage(m_turnPID.calculate(getAngle().getRadians(), m_turnAngleSetpoint.getRadians()));
        }

        if (m_driveWheelSpeedSetpointRadPerSec != null) {
            m_io.setDriveVoltage(
                m_drivePID.calculate(
                    m_inputs.driveVelocityRadPerSec,
                    m_driveFeedforward.calculate(m_driveWheelSpeedSetpointRadPerSec) + (m_turnAngleSetpoint != null
                        ? m_driveWheelSpeedSetpointRadPerSec * Math.cos(m_turnPID.getPositionError())
                        : m_driveWheelSpeedSetpointRadPerSec)
                )
            );
        }
    }
}
