package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants.SwerveModuleConstants;
import frc.robot.util.InvalidSwerveModuleMotorConfigurationException;

public class SwerveSubsystem extends SubsystemBase {
	private SwerveModule[] m_swerveModules;

	public SwerveSubsystem() {
		try {
			switch (Constants.kCurrentMode) {
				case REAL:
				case REPLAY:
					m_swerveModules = new SwerveModule[] {
						new SwerveModule(SwerveModuleConstants.kFrontLeftRealModuleConstants),
						new SwerveModule(SwerveModuleConstants.kFrontRightRealModuleConstants),
						new SwerveModule(SwerveModuleConstants.kBackLeftRealModuleConstants),
						new SwerveModule(SwerveModuleConstants.kBackRightRealModuleConstants)
					};

					break;
				case SIM:
					m_swerveModules = new SwerveModule[] {
						new SwerveModule(SwerveModuleConstants.kFrontLeftSimModuleConstants),
						new SwerveModule(SwerveModuleConstants.kFrontRightSimModuleConstants),
						new SwerveModule(SwerveModuleConstants.kBackLeftSimModuleConstants),
						new SwerveModule(SwerveModuleConstants.kBackRightSimModuleConstants)
					};

					break;
			}

		}
		catch (InvalidSwerveModuleMotorConfigurationException e) {
			DriverStation.reportError("InvalidSwerveModuleMotorConfiguration", e.getStackTrace());
		}
	}
}
