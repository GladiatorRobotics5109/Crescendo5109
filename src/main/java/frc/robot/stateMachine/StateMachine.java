package frc.robot.stateMachine;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.swerve.SwerveSubsystem;

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

    public static void init(SwerveSubsystem swerve) {
        SwerveState.init(swerve);
    }

    public static void periodic() {
        SwerveState.periodic();
    }

    /**
     * Singleton that provides static methods to access the state of the swerve
     * subsystem
     */
    public static final class SwerveState {
        /**
         * Represents the wether or not the bot is driving
         */
        public static enum SwerveDrivingState {
            STOPPED_COASTING, STOPPED_BRAKING, DRIVING_JOYSTICK, DRIVING_FOLLOWING_PATH, DRIVING_UNKNOWN
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

        public static ChassisSpeeds getChassisSpeeds() {
            return s_instance.m_swerve.getChassisSpeeds();
        }

        public static SwerveDrivingState getDrivingState() {
            return s_instance.m_swerve.getDrivingState();
        }

        public static boolean isDriving() {
            return s_instance.m_swerve.isDriving();
        }

        public static boolean isStopped() {
            return s_instance.m_swerve.isStopped();
        }

        private static void periodic() {
            Logger.recordOutput("SwerveState/RobotPose", getPose());
            Logger.recordOutput("SwerveState/ModuleStates", getModuleStates());
            Logger.recordOutput("SwerveState/ChassisSpeeds", getChassisSpeeds());
            Logger.recordOutput("SwerveState/DrivingState", getDrivingState());
            Logger.recordOutput("SwerveState/IsDriving", isDriving());
            Logger.recordOutput("SwerveState/IsStopped", isStopped());
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
}
