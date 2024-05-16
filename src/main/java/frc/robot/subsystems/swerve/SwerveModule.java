package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants.SwerveModuleConstants;
import frc.robot.util.InvalidSwerveModuleMotorConfigurationException;
import frc.robot.util.MotorController;

public class SwerveModule {
	private final SwerveModuleIO m_io;
	private final SwerveModuleIOInputsAutoLogged m_inputs;
	private final int m_index;

	private final SimpleMotorFeedforward m_driveFeedforward;
	private final PIDController m_drivePID;
	private final PIDController m_turnPID;

	private Rotation2d m_turnAngleSetpoint;
	private Double m_driveSpeedSetpoint;
	private Rotation2d m_turnRelativeOffset;
	private SwerveModulePosition[] m_odometryPositions;

	private Translation2d m_modulePosition;

	public SwerveModule(SwerveModuleConstants constants) throws InvalidSwerveModuleMotorConfigurationException {
		MotorController driveController = constants.driveMotorController;
		MotorController turnController = constants.turnMotorController;

		if (driveController == MotorController.TalonFX && turnController == MotorController.SparkMAX) {
			m_io = new SwerveModuleIOTalonFXDriveSparkMaxTurn();
		}
		else if (
			driveController == MotorController.Sim && turnController == MotorController.Sim
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

		m_turnAngleSetpoint = null;
		m_driveSpeedSetpoint = null;

		m_turnRelativeOffset = constants.turnRelativeOffset;

		m_odometryPositions = new SwerveModulePosition[] {};
	}
	
	public void setDesiredState(SwerveModuleState desiredState) {
		setDesiredState(desiredState, true);
	}
	
	public void setDesiredState(SwerveModuleState desiredState, boolean optimize) {
		
	}
}
