package frc.robot;

import org.gladiatorrobotics.gladiatorroboticslib.MotorControllerType;
import org.gladiatorrobotics.gladiatorroboticslib.advantagekitutil.Mode;
import org.gladiatorrobotics.gladiatorroboticslib.constants.swerveModuleConstants.SwerveDriveSpecialtiesConstants.MK4Constants;
import org.gladiatorrobotics.gladiatorroboticslib.constants.swerveModuleConstants.SwerveDriveSpecialtiesConstants.MK4Constants.MK4GearRatio;
import org.gladiatorrobotics.gladiatorroboticslib.math.controller.PIDConstants;
import org.gladiatorrobotics.gladiatorroboticslib.math.controller.SimpleMotorFeedforwardConstants;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.Conversions;

public final class Constants {
    public static final Measure<Time> kRobotLoopPeriod = Units.Seconds.of(Robot.defaultPeriodSecs);
    public static final double kRobotLoopPeriodSecs = kRobotLoopPeriod.in(Units.Seconds);

    /**
     * Alliance to be assumed if DriverStation.getAllliance() is empty
     */
    public static final Alliance kDefaultAlliance = Alliance.Blue;

    public static final Mode kCurrentMode = Mode.SIM;

    public static final class DriveTeamConstants {
        public static final int kDriverControllerPort = 0;

        public static final double kDriveJoystickDeadzone = 0.15;
        public static final double kDriveJoystickTranslationRateLimit = 20;
        public static final double kDriveJoystickAngularRateLimit = 10;

        public static final double kDriverOnNoteEnterRumbleDurationSecs = 0.5;
    }

    public static final class TeleopConstants {
        public static final boolean kDriveFieldRelative = true;
    }

    public static final class SwerveConstants {
        public static final double kOdometryFrequency = 250.0;

        public static final Measure<Distance> kDriveBaseRadius = Units.Meters.of(
            SwerveModuleConstants.kModulePosFL.getNorm()
        );

        public static final Measure<Velocity<Distance>> kDefaultSpeed = Units.MetersPerSecond.of(3.8);
        public static final Measure<Velocity<Angle>> kDefaultAngularSpeed = Units.RadiansPerSecond.of(1.5 * Math.PI);

        /**
         * Offset the desired rotational velocity calculated by the heading PID controller in the
         * {@link frc.robot.subsystems.swerve.SwerveSubsystem} based on the chassis velocity, useful for shooting while
         * moving
         */
        public static final boolean kHeadingControlVelocityCompensation = true;
        // TODO: Scalar should also be scaled by distance from target point, but heading targeting doesn't target a
        // point (it targets a heading) so this might be a large change/addition.
        public static final double kHeadingControlVelocityCompensationScalar = 0.65;
        public static final PIDConstants kSimHeadingPID = new PIDConstants(
            8,
            15,
            0,
            Conversions.degreesToRadians(18),
            true,
            0,
            2 * Math.PI,
            Conversions.degreesToRadians(2),
            Conversions.degreesToRadians(25)
        );
        // TODO: Figure out correct constants for heading control
        public static final PIDConstants kRealHeadingPID = kSimHeadingPID;

        // Wether or not to feed estimated pose from cameras in to the subsystem's pose estimator
        public static final boolean kUseSimCameraForPoseEstimation = false;

        public static final HolonomicPathFollowerConfig kHolonomicPathFollowerConfig = new HolonomicPathFollowerConfig(
            SwerveModuleConstants.kMaxAttainableSpeed.in(Units.MetersPerSecond),
            kDriveBaseRadius.in(Units.Meters),
            new ReplanningConfig(true, true, 0.01, 0.01)
        );

        public static final class SwerveModuleConstants {
            // TODO: tune all constants
            public static final double kWheelRadiusMeters = 0.0472659347214289;

            // Positions of every swerve module
            public static final Translation2d kModulePosFL = new Translation2d(0.290449, 0.290449);
            public static final Translation2d kModulePosFR = new Translation2d(0.290449, -0.290449);
            public static final Translation2d kModulePosBL = new Translation2d(-0.290449, 0.290449);
            public static final Translation2d kModulePosBR = new Translation2d(-0.290449, -0.290449);

            // SDS MK4 L1 gear ratios
            public static final MK4GearRatio kDriveGearRatioEnum = MK4GearRatio.L1;
            public static final double kDriveGearRatio = MK4Constants.getDriveGearRatio(kDriveGearRatioEnum); // 8.14:1
            public static final double kTurnGearRatio = MK4Constants.kTurnGearRatio; // 12.8:1
            public static final Measure<Velocity<Distance>> kMaxAttainableSpeed = Units.FeetPerSecond.of(12.9);

            public static final SimpleMotorFeedforwardConstants kRealFeedForwardConstants = new SimpleMotorFeedforwardConstants(
                0.1,
                0.13,
                0
            );

            public static final SimpleMotorFeedforwardConstants kSimFeedForwardConstants = new SimpleMotorFeedforwardConstants(
                0,
                0.13,
                0
            );

            public static final PIDConstants kRealDrivePID = new PIDConstants(0.5, 0.0, 0.0);
            public static final PIDConstants kRealTurnPID = new PIDConstants(7.0, 0, 0);

            public static final PIDConstants kSimDrivePID = new PIDConstants(1.4, 0.0, 0.0);
            public static final PIDConstants kSimTurnPID = new PIDConstants(10.0, 0, 0);

            public static final SwerveModuleConstants kFrontLeftRealModuleConstants = new SwerveModuleConstants(
                0,
                kRealFeedForwardConstants,
                kRealDrivePID,
                kRealTurnPID,
                kModulePosFL,
                Rotation2d.fromRadians(0), // turn relative offset
                MotorControllerType.TalonFX,
                MotorControllerType.SparkMAX,
                11,
                12,
                Rotation2d.fromRotations(0) // absolute encoder offset
            );

            public static final SwerveModuleConstants kFrontRightRealModuleConstants = new SwerveModuleConstants(
                1,
                kRealFeedForwardConstants,
                kRealDrivePID,
                kRealTurnPID,
                kModulePosFR,
                Rotation2d.fromRadians(0),
                MotorControllerType.TalonFX,
                MotorControllerType.SparkMAX,
                13,
                14,
                Rotation2d.fromRotations(0)
            );

            public static final SwerveModuleConstants kBackLeftRealModuleConstants = new SwerveModuleConstants(
                2,
                kRealFeedForwardConstants,
                kRealDrivePID,
                kRealTurnPID,
                kModulePosBL,
                Rotation2d.fromRadians(0),
                MotorControllerType.TalonFX,
                MotorControllerType.SparkMAX,
                15,
                16,
                Rotation2d.fromRotations(0)
            );

            public static final SwerveModuleConstants kBackRightRealModuleConstants = new SwerveModuleConstants(
                3,
                kRealFeedForwardConstants,
                kRealDrivePID,
                kRealTurnPID,
                kModulePosBR,
                Rotation2d.fromRadians(0),
                MotorControllerType.TalonFX,
                MotorControllerType.SparkMAX,
                17,
                18,
                Rotation2d.fromRotations(0)
            );

            public static final SwerveModuleConstants kFrontLeftSimModuleConstants = new SwerveModuleConstants(
                0,
                kSimFeedForwardConstants,
                kSimDrivePID,
                kSimTurnPID,
                kModulePosFL,
                Rotation2d.fromRadians(0),
                MotorControllerType.Sim,
                MotorControllerType.Sim,
                0,
                0,
                Rotation2d.fromRotations(0)
            );

            public static final SwerveModuleConstants kFrontRightSimModuleConstants = new SwerveModuleConstants(
                1,
                kSimFeedForwardConstants,
                kSimDrivePID,
                kSimTurnPID,
                kModulePosFR,
                Rotation2d.fromRadians(0),
                MotorControllerType.Sim,
                MotorControllerType.Sim,
                0,
                0,
                Rotation2d.fromRotations(0)
            );

            public static final SwerveModuleConstants kBackLeftSimModuleConstants = new SwerveModuleConstants(
                2,
                kSimFeedForwardConstants,
                kSimDrivePID,
                kSimTurnPID,
                kModulePosBL,
                Rotation2d.fromRadians(0),
                MotorControllerType.Sim,
                MotorControllerType.Sim,
                0,
                0,
                Rotation2d.fromRotations(0)
            );

            public static final SwerveModuleConstants kBackRightSimModuleConstants = new SwerveModuleConstants(
                3,
                kSimFeedForwardConstants,
                kSimDrivePID,
                kSimTurnPID,
                kModulePosBR,
                Rotation2d.fromRadians(0),
                MotorControllerType.Sim,
                MotorControllerType.Sim,
                0,
                0,
                Rotation2d.fromRotations(0)
            );

            // Creates empty SwerveModuleConstants because we don't want an io
            // implementation in replay
            public static final SwerveModuleConstants kReplayModuleConstants = new SwerveModuleConstants(Mode.REPLAY);

            public int index;

            public SimpleMotorFeedforwardConstants driveFeedforward;
            public PIDConstants drivePID;
            public PIDConstants turnPID;

            public Translation2d moduleTranslation2d;

            public Rotation2d turnRelativeOffset;

            public MotorControllerType driveMotorController;
            public MotorControllerType turnMotorController;

            public int driveMotorPort;
            public int turnMotorPort;

            public Rotation2d turnAbsoluteEncoderOffset;

            /**
             * Empty SwerveModuleConstants (for replaying a log file with no io impl)
             */
            public SwerveModuleConstants(Mode mode) {
                if (mode != Mode.REPLAY) {
                    DriverStation.reportWarning(
                        "Empty SwerveModuleConstants created for a robot that is not in Mode.REPLAY",
                        false
                    );
                }
            }

            public SwerveModuleConstants(
                int index,
                SimpleMotorFeedforwardConstants driveFeedforward,
                PIDConstants drivePID,
                PIDConstants turnPID,
                Translation2d modulePosition,
                Rotation2d turnRelativeOffset,
                MotorControllerType driveMotorController,
                MotorControllerType turnMotorController,
                int driveMotorPort,
                int turnMotorPort,
                Rotation2d turnAbsoluteEncoderOffset
            ) {
                this.index = index;
                this.driveFeedforward = driveFeedforward;
                this.drivePID = drivePID;
                this.turnPID = turnPID;
                this.moduleTranslation2d = modulePosition;
                this.turnRelativeOffset = turnRelativeOffset;
                this.driveMotorController = driveMotorController;
                this.turnMotorController = turnMotorController;
                this.driveMotorPort = driveMotorPort;
                this.turnMotorPort = turnMotorPort;
                this.turnAbsoluteEncoderOffset = turnAbsoluteEncoderOffset;
            }
        }

        public static final class VisionConstants {
            public static final AprilTagFields kAprilTagFieldLayout = AprilTagFields.k2024Crescendo;
            public static final PoseStrategy kPoseEstimationStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

            public static final String[] kRealCameraNames = new String[] {
                "LeftCamera",
                "RightCamera"
            };
            public static final Transform3d[] kRealRobotToCameras = new Transform3d[] {
                new Transform3d(
                    0.17145,
                    0.152654,
                    0.107442,
                    new Rotation3d(0, Conversions.degreesToRadians(-15), 0)
                ),
                new Transform3d(
                    0.17145,
                    -0.152654,
                    0.107442,
                    new Rotation3d(0, Conversions.degreesToRadians(-15), 0)
                )
            };

            // Simulate real cameras using PhotonVision's simulator
            public static final String[] kSimCameraNames = kRealCameraNames;
            public static final Transform3d[] kSimRobotToCameras = kRealRobotToCameras;
            // public static final String[] kSimCameraNames = new String[] {};
            // public static final Transform3d[] kSimRobotToCameras = new Transform3d[] {};

            // TODO: verify these values with real bot sometime
            public static final class SimCameraConstants {
                public static final double kAverageLatencyMs = 33.3333333333; // ~30 fps
                // TODO: Measure this
                public static final double kLatencyStdDevMs = 5;
                public static final double kFPS = 120.0;

                public static final int kImageWidth = 1280;
                public static final int kImageHeight = 800;
                // TODO: Make sure diag fov is correct
                public static final Rotation2d kFov = Rotation2d.fromDegrees(86.0510613313);
            }
        }
    }

    public static final class WinchConstants {
        public static final int kRealMotorPort = 5;
        public static final int kRealMotorCurrentLimit = 40;
        public static final PIDConstants kRealAnglePID = new PIDConstants(1.0, 0.0, 0.0);

        public static final PIDConstants kSimAnglePID = new PIDConstants(
            200,
            0,
            0,
            Double.POSITIVE_INFINITY,
            false,
            Double.NEGATIVE_INFINITY,
            Double.POSITIVE_INFINITY,
            Conversions.degreesToRadians(1),
            Conversions.degreesToRadians(4)
        );

        public static final double kPivotWinchAverageRadius = 0.75 / 2;

        public static final double kRealWinchGearRatio = 25;
        // Use a different gear ratio than real bc the sim motors are unrealistically slow for some reason
        public static final double kSimWinchGearRatio = 5;

        // public static final double kWinchPositionConversionFactor = kPivotWinchAverageRadius * (2 * Math.PI) /
        // kWinchGearRatio;

        public static final Rotation2d kMaxAngle = Rotation2d.fromDegrees(59);
        public static final Rotation2d kMinAngle = Rotation2d.fromDegrees(30);
        public static final Rotation2d kStartingAngle = kMinAngle;

        public static final Rotation2d kIntakeAngle = kMinAngle;
    }

    public static final class ShooterConstants {
        public static final double kRPMTolerance = 50.0;

        public static final double kAutoSpinUpRadiusMeters = 8.0;
        public static final double kShootRPM = 5500.0;
        public static final double kAutoSpinRPM = kShootRPM;
    }

    public static final class RollersConstants {
        public static final class IntakeConstants {
            public static final int kRealMotorPort = 0;

            public static final int kRealCurrentLimitAmps = 40;

            public static final PIDConstants kRealRPMPID = new PIDConstants(1, 0, 0);
            public static final PIDConstants kSimRPMPID = kRealRPMPID;

            public static final double kIntakeRPM = 100.0;

            public static final int kNoteEnterCurrentThreshold = 10;

            public static final double kIntakeGearRatio = 5;
        }

        public static final class FeederConstants {
            public static final int kRealMotorPort = 4;

            public static final int kRealCurrentLimitAmps = 40;

            public static final PIDConstants kRealRPMPID = new PIDConstants(1.0, 0.0, 0.0);
            public static final PIDConstants kSImRPMPID = kRealRPMPID;

            public static final int kNoteEnterCurrentThreshold = 10;

            public static final double kFeedRPM = 100;

            public static final int kNoteSensorChannel = 0;
        }
    }
}
