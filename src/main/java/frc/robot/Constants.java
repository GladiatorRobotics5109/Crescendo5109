package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.SimpleMotorFeedforwardConstants;
import frc.robot.util.MotorControllerType;
import frc.robot.util.PIDConstants;

public final class Constants {
    public static final Measure<Time> kRobotLoopPeriod = Units.Seconds.of(Robot.defaultPeriodSecs);
    public static final double kRobotLoopPeriodSecs = kRobotLoopPeriod.in(Units.Seconds);

    /**
     * Alliance to be assumed if DriverStation.getAllliance() is empty
     */
    public static final Alliance kDefaultAlliance = Alliance.Blue;

    public static final Mode kCurrentMode = Mode.SIM;

    public enum Mode {
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

    public static final class DriveTeamConstants {
        public static final int kDriverControllerPort = 0;

        public static final double kDriveJoystickDeadzone = 0.15;
        public static final double kDriveJoystickTranslationRateLimit = 20;
        public static final double kDriveJoystickAngularRateLimit = 10;
    }

    public static final class TeleopConstants {
        public static final boolean kDriveFieldRelative = false;
    }

    public static final class SwerveConstants {
        public static final class SwerveModuleConstants {
            // TODO: tune all constants
            public static final Measure<Velocity<Distance>> kMaxAttainableSpeed = Units.FeetPerSecond.of(12.9);

            public static final double kWheelRadiusMeters = 0.0472659347214289;

            // Positions of every swerve module
            public static final Translation2d kModulePosFL = new Translation2d(0.290449, 0.290449);
            public static final Translation2d kModulePosFR = new Translation2d(0.290449, -0.290449);
            public static final Translation2d kModulePosBL = new Translation2d(-0.290449, 0.290449);
            public static final Translation2d kModulePosBR = new Translation2d(-0.290449, -0.290449);

            // SDS MK4 L1 gear ratios
            public static final double kDriveGearRatio = 8.14; // 8.14:1
            public static final double kTurnGearRatio = 12.8; // 12.8:1

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

            public static final PIDConstants kSimDrivePID = new PIDConstants(1.4, 0.0, 0.0);
            public static final PIDConstants kSimTurnPID = new PIDConstants(10.0, 0, 0);

            public static final SwerveModuleConstants kFrontLeftRealModuleConstants = new SwerveModuleConstants(
                0,
                kRealFeedForwardConstants,
                kRealDrivePID,
                kRealTurnPID,
                kModulePosFL,
                Rotation2d.fromRadians(0),
                MotorControllerType.TalonFX,
                MotorControllerType.SparkMAX,
                0,
                0,
                Rotation2d.fromRotations(0)
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
                0,
                0,
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
                0,
                0,
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
                0,
                0,
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
             *
             * @param mode
             */
            public SwerveModuleConstants(Mode mode) {
                if (mode != Mode.REPLAY) {
                    DriverStation.reportWarning(
                        "Empty SwerveModuleConstants created for a robot that is not in Mode.REPLAY", null
                    );
                }
            }

            public SwerveModuleConstants(
                int index, SimpleMotorFeedforwardConstants driveFeedforward, PIDConstants drivePID,
                PIDConstants turnPID, Translation2d modulePosition, Rotation2d turnRelativeOffset,
                MotorControllerType driveMotorController, MotorControllerType turnMotorController, int driveMotorPort,
                int turnMotorPort, Rotation2d turnAbsoluteEncoderOffset
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

        public static final double kOdometryFrequency = 250.0;

        public static final Measure<Velocity<Distance>> kDefaultSpeed = Units.MetersPerSecond.of(3.5);
        public static final Measure<Velocity<Angle>> kDefaultAngularSpeed = Units.RadiansPerSecond.of(Math.PI);
    }
}
