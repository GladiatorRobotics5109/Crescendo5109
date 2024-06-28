package frc.robot.stateMachine;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionMeasurement;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Static class that represents the state of the robot.
 * <ul>
 * <li>State is updated in every subsystem, this class (and its subclasses)
 * provide a way to statically access the state of every subsystem.
 * <li>A subsystem <b>should not</b> read its state through its corresponding
 * state machine methods.
 * </ul>
 */
public final class StateMachine {
    private StateMachine() {
        throw new UnsupportedOperationException("This class should not be instantiated!");
    }

    public static void init(VisionSubsystem vision, SwerveSubsystem swerve) {
        VisionState.init(vision);
        SwerveState.init(swerve);
    }

    public static void periodic() {
        SwerveState.periodic();
        VisionState.periodic();
    }

    /**
     * Singleton that provides static methods to access the state of the swerve
     * subsystem
     */
    public static final class SwerveState {
        /**
         * Represents if and how the drivetrain is moving
         */
        public static enum SwerveDrivingState {
            STOPPED_COASTING,
            /** The drivetrain is stopped and the modules are coasting */
            STOPPED_BRAKING,
            /** The drivetrain is stopped and the modules are braking */
            MOVING_COASTING,
            /** The drivetrain is moving and the modules are coasting */
            MOVING_BRAKING,
            /** The drivetrain is moving and the modules are braking */
            DRIVING_JOYSTICK,
            /** The drivetrain is moving and it is being driven with a joystick */
            DRIVING_FOLLOWING_PATH,
            /** The drivetrain is moving and it is being driven by a path follower */
            DRIVING_UNKNOWN
            /** The drivetrain is moving and it is being driven by an unknown source */
        }

        public static Pose2d getPose() {
            return s_instance.m_swerve.getPose();
        }

        public static Rotation2d getHeading() {
            return s_instance.m_swerve.getHeading();
        }

        public static SwerveModuleState[] getModuleStates() {
            return s_instance.m_swerve.getModuleStates();
        }

        public static ChassisSpeeds getRobotRelativeChassisSpeeds() {
            return s_instance.m_swerve.getRobotRelativeChassisSpeeds();
        }

        public static ChassisSpeeds getFieldRelativeChassisSpeeds() {
            return s_instance.m_swerve.getFieldRelativeChassisSpeeds();
        }

        public static SwerveDrivingState getDrivingState() {
            return s_instance.m_swerve.getDrivingState();
        }

        public static boolean isDriving() {
            return s_instance.m_swerve.isDriving();
        }

        public static boolean isMoving() {
            return s_instance.m_swerve.isMoving();
        }

        public static boolean isStopped() {
            return s_instance.m_swerve.isStopped();
        }

        public static boolean isTargetingHeading() {
            return s_instance.m_swerve.isTargetingHeading();
        }

        public static boolean isAtTargetHeading() {
            return s_instance.m_swerve.isAtTargetHeading();
        }

        public static Measure<Velocity<Distance>> getCurrentMaxSpeed() {
            return s_instance.m_swerve.getCurrentMaxSpeed();
        }

        private static void periodic() {
            Logger.recordOutput("SwerveState/RobotPose", getPose());
            Logger.recordOutput("SwerveState/ModuleStates", getModuleStates());
            Logger.recordOutput("SwerveState/RobotRelativeChassisSpeeds", getRobotRelativeChassisSpeeds());
            Logger.recordOutput("SwerveState/FieldRelativeChassisSpeeds", getFieldRelativeChassisSpeeds());
            Logger.recordOutput("SwerveState/DrivingState", getDrivingState());
            Logger.recordOutput("SwerveState/IsDriving", isDriving());
            Logger.recordOutput("SwerveState/IsMoving", isMoving());
            Logger.recordOutput("SwerveState/IsStopped", isStopped());
            Logger.recordOutput("SwerveState/IsTargetingHeading", isTargetingHeading());
            Logger.recordOutput("SwerveState/IsAtTargetHeading", isAtTargetHeading());
            Logger.recordOutput("SwerveState/CurrentMaxSpeed", getCurrentMaxSpeed());
        }

        private static SwerveState s_instance;

        private static void init(SwerveSubsystem swerve) {
            s_instance = new SwerveState(swerve);
        }

        private SwerveSubsystem m_swerve;

        private SwerveState(SwerveSubsystem swerve) {
            m_swerve = swerve;
        }
    }

    public static final class VisionState {
        public static VisionMeasurement[] getMeasurements() {
            return s_instance.m_vision.getMeasurements();
        }

        private static void periodic() {
            VisionMeasurement[] measurements = getMeasurements();
            for (VisionMeasurement measurement : measurements) {
                Logger.recordOutput(
                    "VisionState/" + measurement.getCameraName() + "Measurement",
                    measurement
                );
            }
        }

        private static VisionState s_instance;

        private static void init(VisionSubsystem vision) {
            s_instance = new VisionState(vision);
        }

        private VisionSubsystem m_vision;

        private VisionState(VisionSubsystem vision) {
            m_vision = vision;
        }
    }
}
