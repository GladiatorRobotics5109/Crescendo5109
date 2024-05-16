package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import frc.robot.util.SimpleMotorFeedforwardConstants;
import frc.robot.util.MotorController;
import frc.robot.util.PIDConstants;

public final class Constants {
	public static final Measure<Time> kRobotLoopPeriod = Units.Seconds.of(Robot.defaultPeriodSecs);

	public static final Mode kCurrentMode = Mode.SIM;

	public static enum Mode {
		/**
		 * Running on a real robot.
		 */
		REAL,

		/**
		 * Running a physics simulator.
		 */
		SIM,

		/**
		 * Replaying from a log file.
		 */
		REPLAY
	}

	public static final class SwerveConstants {
		public static final class SwerveModuleConstants {
			public static final SimpleMotorFeedforwardConstants kRealFeedForwardConstants = new SimpleMotorFeedforwardConstants(
				0.1,
				0.13,
				0
			);

			public static final SimpleMotorFeedforwardConstants kSimFeedForwardConstants = new SimpleMotorFeedforwardConstants(
				0, 0.13, 0
			);

			public static final PIDConstants kRealDrivePID = new PIDConstants(0.5, 0.0, 0.0);
			public static final PIDConstants kRealTurnPID = new PIDConstants(7.0, 0, 0);

			public static final PIDConstants kSimDrivePID = new PIDConstants(0.1, 0.0, 0.0);
			public static final PIDConstants kSimTurnPID = new PIDConstants(10.0, 0, 0);

			public static final SwerveModuleConstants kFrontLeftRealModuleConstants = new SwerveModuleConstants(
				0,
				kRealFeedForwardConstants,
				kRealDrivePID,
				kRealTurnPID,
				new Translation2d(0.290449, 0.290449),
				Rotation2d.fromRadians(0),
				MotorController.TalonFX,
				MotorController.SparkMAX
			);

			public static final SwerveModuleConstants kFrontRightRealModuleConstants = new SwerveModuleConstants(
				1,
				kRealFeedForwardConstants,
				kRealDrivePID,
				kRealTurnPID,
				new Translation2d(0.290449, -0.290449),
				Rotation2d.fromRadians(0),
				MotorController.TalonFX,
				MotorController.SparkMAX
			);

			public static final SwerveModuleConstants kBackLeftRealModuleConstants = new SwerveModuleConstants(
				2,
				kRealFeedForwardConstants,
				kRealDrivePID,
				kRealTurnPID,
				new Translation2d(-0.290449, 0.290449),
				Rotation2d.fromRadians(0),
				MotorController.TalonFX,
				MotorController.SparkMAX
			);

			public static final SwerveModuleConstants kBackRightRealModuleConstants = new SwerveModuleConstants(
				3,
				kRealFeedForwardConstants,
				kRealDrivePID,
				kRealTurnPID,
				new Translation2d(-0.290449, -0.290449),
				Rotation2d.fromRadians(0),
				MotorController.TalonFX,
				MotorController.SparkMAX
			);

			public static final SwerveModuleConstants kFrontLeftSimModuleConstants = new SwerveModuleConstants(
				0,
				kSimFeedForwardConstants,
				kSimDrivePID,
				kSimTurnPID,
				new Translation2d(0.290449, 0.290449),
				Rotation2d.fromRadians(0),
				MotorController.Sim,
				MotorController.Sim
			);

			public static final SwerveModuleConstants kFrontRightSimModuleConstants = new SwerveModuleConstants(
				1,
				kSimFeedForwardConstants,
				kSimDrivePID,
				kSimTurnPID,
				new Translation2d(0.290449, -0.290449),
				Rotation2d.fromRadians(0),
				MotorController.Sim,
				MotorController.Sim
			);

			public static final SwerveModuleConstants kBackLeftSimModuleConstants = new SwerveModuleConstants(
				2,
				kSimFeedForwardConstants,
				kSimDrivePID,
				kSimTurnPID,
				new Translation2d(-0.290449, 0.290449),
				Rotation2d.fromRadians(0),
				MotorController.Sim,
				MotorController.Sim
			);

			public static final SwerveModuleConstants kBackRightSimModuleConstants = new SwerveModuleConstants(
				3,
				kSimFeedForwardConstants,
				kSimDrivePID,
				kSimTurnPID,
				new Translation2d(-0.290449, -0.290449),
				Rotation2d.fromRadians(0),
				MotorController.Sim,
				MotorController.Sim
			);

			public int index;

			public SimpleMotorFeedforwardConstants driveFeedforward;
			public PIDConstants drivePID;
			public PIDConstants turnPID;

			public Translation2d modulePosition;

			public Rotation2d turnRelativeOffset;

			public MotorController driveMotorController;
			public MotorController turnMotorController;

			public SwerveModuleConstants() {

			}

			public SwerveModuleConstants(
				int index, SimpleMotorFeedforwardConstants driveFeedforward, PIDConstants drivePID,
				PIDConstants turnPID, Translation2d modulePosition, Rotation2d turnRelativeOffset,
				MotorController driveMotorController, MotorController turnMotorController
			) {
				this.index = index;
				this.driveFeedforward = driveFeedforward;
				this.drivePID = drivePID;
				this.turnPID = turnPID;
				this.modulePosition = modulePosition;
				this.turnRelativeOffset = turnRelativeOffset;
				this.driveMotorController = driveMotorController;
				this.turnMotorController = turnMotorController;
			}
		}
	}
}
