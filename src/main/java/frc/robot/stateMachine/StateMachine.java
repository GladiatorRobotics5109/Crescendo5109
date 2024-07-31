package frc.robot.stateMachine;

import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.winch.WinchSubsystem;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import frc.robot.subsystems.shooter.ShooterSubsystem;
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

    public static void init(
        SwerveSubsystem swerve,
        VisionSubsystem vision,
        ShooterSubsystem shooter,
        WinchSubsystem winch,
        Rollers rollers
    ) {
        SwerveState.init(swerve);
        VisionState.init(vision);
        ShooterState.init(shooter);
        WinchState.init(winch);
        RollersState.init(rollers);
        RobotState.init();
    }

    public static void periodic() {
        SwerveState.periodic();
        VisionState.periodic();
        ShooterState.periodic();
        WinchState.periodic();
        RollersState.periodic();
    }

    public static final class RobotState {
        public static boolean isAssistedShooting() {
            return s_instance.m_isAssistedShooting;
        }

        public static void setIsAssistedShooting(boolean isAssistedShooting) {
            s_instance.m_isAssistedShooting = isAssistedShooting;
        }

        private static RobotState s_instance;

        private static void init() {
            s_instance = new RobotState();
        }

        private boolean m_isAssistedShooting;

        private RobotState() {
            m_isAssistedShooting = false;
        }
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

    public static final class ShooterState {
        public static double getLeftCurrentRPM() {
            return s_instance.m_shooter.getLeftCurrentRPM();
        }

        public static double getRightCurrentRPM() {
            return s_instance.m_shooter.getRightCurrentRPM();
        }

        public static double getLeftDesiredRPM() {
            return s_instance.m_shooter.getLeftDesiredRPM();
        }

        public static double getRightDesiredRPM() {
            return s_instance.m_shooter.getRightDesiredRPM();
        }

        public static boolean getAutoSpinUpEnabled() {
            return s_instance.m_shooter.getAutoSpinUpEnabled();
        }

        public static boolean isSpinning() {
            return s_instance.m_shooter.isSpinning();
        }

        public static boolean shouldAutoSpinUp() {
            return s_instance.m_shooter.shouldAutoSpinUp();
        }

        public static boolean hasDesiredRPM() {
            return s_instance.m_shooter.hasDesiredRPM();
        }

        public static boolean isAtDesiredRPM() {
            return s_instance.m_shooter.isAtDesiredRPM();
        }

        public static void periodic() {
            Logger.recordOutput("ShooterState/LeftCurrentRPM", getLeftCurrentRPM());
            Logger.recordOutput("ShooterState/RightCurrentRPM", getRightCurrentRPM());
            Logger.recordOutput("ShooterState/LeftDesiredRPM", getLeftDesiredRPM());
            Logger.recordOutput("ShooterState/RightDesiredRPM", getRightDesiredRPM());
            Logger.recordOutput("ShooterState/AutoSpinUpEnabled", getAutoSpinUpEnabled());
            Logger.recordOutput("ShooterState/ShouldAutoSpinUp", shouldAutoSpinUp());
            Logger.recordOutput("ShooterState/IsSpinning", isSpinning());
            Logger.recordOutput("ShooterState/HasDesiredRPM", hasDesiredRPM());
            Logger.recordOutput("ShooterState/IsAtDesiredRPM", isAtDesiredRPM());
        }

        private static ShooterState s_instance;

        private static void init(ShooterSubsystem shooter) {
            s_instance = new ShooterState(shooter);
        }

        private ShooterSubsystem m_shooter;

        private ShooterState(ShooterSubsystem shooter) {
            m_shooter = shooter;
        }
    }

    public static final class WinchState {
        public static Rotation2d getCurrentAngle() {
            return s_instance.m_winch.getCurrentAngle();
        }

        public static Rotation2d getTargetAngle() {
            return s_instance.m_winch.getTargetAngle();
        }

        public static boolean isAtTargetAngle() {
            return s_instance.m_winch.isAtTargetAngle();
        }

        public static boolean isTargetingAngle() {
            return s_instance.m_winch.isTargetingAngle();
        }

        private static void periodic() {
            Logger.recordOutput("WinchState/CurrentAngle", getCurrentAngle());
            Logger.recordOutput("WinchState/TargetAngle", getTargetAngle());
            Logger.recordOutput("WinchState/IsAtTargetAngle", isAtTargetAngle());
            Logger.recordOutput("WinchState/IsTargetingAngle", isTargetingAngle());
        }

        private static WinchState s_instance;

        private static void init(WinchSubsystem winch) {
            s_instance = new WinchState(winch);
        }

        private final WinchSubsystem m_winch;

        private WinchState(WinchSubsystem winch) {
            m_winch = winch;
        }
    }

    public final static class RollersState {
        public static double getIntakeDesiredRPM() {
            return s_instance.m_rollers.getIntakeDesiredRPM();
        }

        public static double getIntakeCurrentRPM() {
            return s_instance.m_rollers.getIntakeCurrentRPM();
        }

        public static boolean getIntakeIsIntaking() {
            return s_instance.m_rollers.getIntakeIsIntaking();
        }

        public static boolean getIntakeHasNote() {
            return s_instance.m_rollers.getIntakeHasNote();
        }

        public static boolean getFeederHasNote() {
            return s_instance.m_rollers.getFeederHasNote();
        }

        public static double getFeederDesiredRPM() {
            return s_instance.m_rollers.getFeederDesiredRPM();
        }

        public static double getFeederCurrentRPM() {
            return s_instance.m_rollers.getFeederCurrentRPM();
        }

        public static boolean getFeederIsIntaking() {
            return s_instance.m_rollers.getFeederIsIntaking();
        }

        public static boolean hasNote() {
            return s_instance.m_rollers.hasNote();
        }

        public static boolean isIntaking() {
            return s_instance.m_rollers.isIntaking();
        }

        private static void periodic() {
            Logger.recordOutput("RollersState/Intake/DesiredRPM", getIntakeDesiredRPM());
            Logger.recordOutput("RollersState/Intake/CurrentRPM", getIntakeCurrentRPM());
            Logger.recordOutput("RollersState/Intake/IsIntaking", getIntakeIsIntaking());
            Logger.recordOutput("RollersState/Intake/HasNote", getIntakeHasNote());
            Logger.recordOutput("RollersState/Feeder/HasNote", getFeederHasNote());
            Logger.recordOutput("RollersState/Feeder/DesiredRPM", getFeederDesiredRPM());
            Logger.recordOutput("RollersState/Feeder/CurrentRPM", getFeederCurrentRPM());
            Logger.recordOutput("RollersState/Feeder/IsIntaking", getFeederIsIntaking());
            Logger.recordOutput("RollersState/IsIntaking", isIntaking());
            Logger.recordOutput("RollersState/HasNote", hasNote());
        }

        private static RollersState s_instance;

        private static void init(Rollers rollers) {
            s_instance = new RollersState(rollers);
        }

        private final Rollers m_rollers;

        private RollersState(Rollers rollers) {
            m_rollers = rollers;
        }
    }
}
