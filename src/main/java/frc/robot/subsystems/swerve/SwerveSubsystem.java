package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.SwerveModuleConstants;
import frc.robot.stateMachine.StateMachine;
import frc.robot.stateMachine.StateMachine.SwerveState.SwerveDrivingState;
import frc.robot.subsystems.swerve.swerveModule.SwerveModule;
import frc.robot.subsystems.vision.VisionMeasurement;
import frc.robot.util.InvalidSwerveModuleMotorConfigurationException;
import frc.robot.util.Util;

import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.github.gladiatorrobotics5109.gladiatorroboticslib.advantagekitutil.loggedgyro.LoggedGyro;
import com.github.gladiatorrobotics5109.gladiatorroboticslib.advantagekitutil.loggedgyro.LoggedGyroIO;
import com.github.gladiatorrobotics5109.gladiatorroboticslib.advantagekitutil.loggedgyro.LoggedGyroIOSim;
import com.github.gladiatorrobotics5109.gladiatorroboticslib.advantagekitutil.loggedpidcontroller.LoggedPIDController;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

public class SwerveSubsystem extends SubsystemBase {
    public static final Lock odometryLock = new ReentrantLock();

    private SwerveModule[] m_swerveModules;

    private LoggedGyro m_gyro;

    private final SwerveDriveKinematics m_kinematics;
    private final SwerveDrivePoseEstimator m_poseEstimator;

    private final SlewRateLimiter m_driverControllerLeftXLimiter;
    private final SlewRateLimiter m_driverControllerLeftYLimiter;
    private final SlewRateLimiter m_driverControllerRightXLimiter;

    private SwerveDrivingState m_drivingState;

    private boolean m_targetHeadingEnabled;
    private final LoggedPIDController m_targetHeadingPID;
    private Supplier<Rotation2d> m_targetHeadingSupplier;

    private Measure<Velocity<Distance>> m_currentMaxSpeed;

    public SwerveSubsystem() {
        try {
            switch (Constants.kCurrentMode) {
                case REAL:
                    m_swerveModules = new SwerveModule[] {
                        new SwerveModule(SwerveModuleConstants.kFrontLeftRealModuleConstants),
                        new SwerveModule(SwerveModuleConstants.kFrontRightRealModuleConstants),
                        new SwerveModule(SwerveModuleConstants.kBackLeftRealModuleConstants),
                        new SwerveModule(SwerveModuleConstants.kBackRightRealModuleConstants)
                    };

                    m_gyro = new LoggedGyro("SwerveInputs/Gyro", new LoggedGyroIOSim());

                    break;
                case SIM:
                    m_swerveModules = new SwerveModule[] {
                        new SwerveModule(SwerveModuleConstants.kFrontLeftSimModuleConstants),
                        new SwerveModule(SwerveModuleConstants.kFrontRightSimModuleConstants),
                        new SwerveModule(SwerveModuleConstants.kBackLeftSimModuleConstants),
                        new SwerveModule(SwerveModuleConstants.kBackRightSimModuleConstants)
                    };

                    m_gyro = new LoggedGyro("SwerveInputs/Gyro", new LoggedGyroIOSim());

                    break;
                default:
                    // m_swerveModules = new SwerveModule[] {
                    // new SwerveModule(SwerveModuleConstants.kReplayModuleConstants),
                    // new SwerveModule(SwerveModuleConstants.kReplayModuleConstants),
                    // new SwerveModule(SwerveModuleConstants.kReplayModuleConstants),
                    // new SwerveModule(SwerveModuleConstants.kReplayModuleConstants)
                    // };

                    m_swerveModules = new SwerveModule[] {
                        new SwerveModule(SwerveModuleConstants.kFrontLeftRealModuleConstants),
                        new SwerveModule(SwerveModuleConstants.kFrontRightRealModuleConstants),
                        new SwerveModule(SwerveModuleConstants.kBackLeftRealModuleConstants),
                        new SwerveModule(SwerveModuleConstants.kBackRightRealModuleConstants)
                    };

                    m_gyro = new LoggedGyro("SwerveInputs/Gyro", new LoggedGyroIO() {});

                    break;
            }

        }
        catch (InvalidSwerveModuleMotorConfigurationException e) {
            DriverStation.reportError("InvalidSwerveModuleMotorConfiguration", e.getStackTrace());
        }

        m_kinematics = new SwerveDriveKinematics(
            m_swerveModules[0].getTranslation2d(), // fl
            m_swerveModules[1].getTranslation2d(), // fr
            m_swerveModules[2].getTranslation2d(), // bl
            m_swerveModules[3].getTranslation2d() // br
        );

        m_poseEstimator = new SwerveDrivePoseEstimator(
            m_kinematics,
            Rotation2d.fromRotations(0),
            getModulePositions(),
            new Pose2d()
        );

        m_driverControllerLeftXLimiter = new SlewRateLimiter(
            Constants.DriveTeamConstants.kDriveJoystickTranslationRateLimit
        );
        m_driverControllerLeftYLimiter = new SlewRateLimiter(
            Constants.DriveTeamConstants.kDriveJoystickTranslationRateLimit
        );
        m_driverControllerRightXLimiter = new SlewRateLimiter(
            Constants.DriveTeamConstants.kDriveJoystickAngularRateLimit
        );

        m_drivingState = SwerveDrivingState.STOPPED_COASTING;

        m_targetHeadingEnabled = false;
        m_targetHeadingPID = SwerveConstants.kSimHeadingPID.getLoggedPIDController("SwerveState/HeadingPID");

        m_currentMaxSpeed = SwerveConstants.kDefaultSpeed;

        // Configure AutoBuilder for PathPlanner Autos
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::setPose,
            this::getRobotRelativeChassisSpeeds,
            (ChassisSpeeds speeds) -> drive(speeds, false),
            SwerveConstants.kHolonomicPathFollowerConfig,
            () -> false,
            this
        );

        // Set PathPlannerLogging callbacks
        PathPlannerLogging.setLogActivePathCallback((List<Pose2d> activePath) -> {
            Pose2d[] arr = new Pose2d[activePath.size()];
            activePath.toArray(arr);
            Logger.recordOutput("SwerveState/PathPlannerActivePath", arr);
        });
        PathPlannerLogging.setLogTargetPoseCallback(
            (Pose2d desiredPose) -> Logger.recordOutput("SwerveState/PathPlannerDesiredPose", desiredPose)
        );
        PathPlannerLogging.setLogCurrentPoseCallback(
            (Pose2d currentPose) -> Logger.recordOutput("SwerveState/PathPlannerCurrentPose", currentPose)
        );
    }

    private void drive(double vx, double vy, double vrot, boolean fieldRelative) {
        drive(new ChassisSpeeds(vx, vy, vrot), fieldRelative);
    }

    /**
     * Drives with the given field/robot relative chassis speeds
     */
    private void drive(ChassisSpeeds desiredSpeeds, boolean fieldRelative) {
        if (m_targetHeadingEnabled && m_targetHeadingSupplier != null) {
            Rotation2d targetHeading = m_targetHeadingSupplier.get();
            ChassisSpeeds currentSpeeds = getFieldRelativeChassisSpeeds();

            // Calculate velocity compensation if enabled
            double velocityCompensation = SwerveConstants.kHeadingControlVelocityCompensation
                ? -SwerveConstants.kHeadingControlVelocityCompensationScalar
                    * ((currentSpeeds.vyMetersPerSecond * Math.cos(targetHeading.getRadians()))
                        + (currentSpeeds.vxMetersPerSecond * -Math.sin(targetHeading.getRadians())))
                : 0;

            desiredSpeeds.omegaRadiansPerSecond = m_targetHeadingPID.calculate(
                getHeading().getRadians(),
                targetHeading.getRadians()
            ) + velocityCompensation;
        }

        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(
            desiredSpeeds,
            Constants.kRobotLoopPeriod.in(Units.Seconds)
        );
        ChassisSpeeds robotRelativeSpeeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(discreteSpeeds, getHeading().plus(getDriveRotationOffset()))
            : discreteSpeeds;
        SwerveModuleState[] desiredStates = m_kinematics.toSwerveModuleStates(robotRelativeSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, m_currentMaxSpeed);

        SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
        for (int i = 0; i < desiredStates.length; i++) {
            optimizedStates[i] = m_swerveModules[i].setDesiredState(desiredStates[i]);
        }

        // Change state if the desired state isn't set to stopping
        if (
            desiredSpeeds.vxMetersPerSecond != 0.0 && desiredSpeeds.vyMetersPerSecond != 0.0
                && desiredSpeeds.omegaRadiansPerSecond != 0.0
        ) {
            m_drivingState = SwerveDrivingState.DRIVING_UNKNOWN;
        }

        Logger.recordOutput("SwerveState/DesiredModuleStates", desiredStates);
        Logger.recordOutput("SwerveState/OptimizedDesiredModuleStatesStates", optimizedStates);
    }

    public void stop() {
        drive(0, 0, 0, true);
    }

    /**
     * @return swerve modules positions
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            m_swerveModules[0].getPosition(), // fl
            m_swerveModules[1].getPosition(), // fr
            m_swerveModules[2].getPosition(), // bl
            m_swerveModules[3].getPosition() // br
        };
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            m_swerveModules[0].getState(),
            m_swerveModules[1].getState(),
            m_swerveModules[2].getState(),
            m_swerveModules[3].getState(),
        };
    }

    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getModuleStates());
    }

    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeChassisSpeeds(), getHeading());
    }

    /**
     * @return what the swerve subsystem is currently doing
     */
    public SwerveDrivingState getDrivingState() {
        return m_drivingState;
    }

    public Measure<Velocity<Distance>> getCurrentMaxSpeed() {
        return m_currentMaxSpeed;
    }

    public boolean isDriving() {
        return m_drivingState == SwerveDrivingState.DRIVING_FOLLOWING_PATH
            || m_drivingState == SwerveDrivingState.DRIVING_JOYSTICK
            || m_drivingState == SwerveDrivingState.DRIVING_UNKNOWN;
    }

    public boolean isMoving() {
        return m_drivingState == SwerveDrivingState.MOVING_BRAKING
            || m_drivingState == SwerveDrivingState.MOVING_COASTING
            || isDriving();
    }

    public boolean isStopped() {
        return m_drivingState == SwerveDrivingState.STOPPED_BRAKING
            || m_drivingState == SwerveDrivingState.STOPPED_COASTING;
    }

    public boolean isTargetingHeading() {
        return m_targetHeadingEnabled && m_targetHeadingSupplier != null;
    }

    public boolean isAtTargetHeading() {
        return isTargetingHeading() && m_targetHeadingPID.atSetpoint();
    }

    /**
     * Resets pose estimator pose
     *
     * @param pose
     *            new robot pose
     */
    public void setPose(Pose2d pose) {
        m_poseEstimator.resetPosition(m_gyro.getYaw(), getModulePositions(), pose);
    }

    public void setTargetHeadingEnabled(boolean enabled, Supplier<Rotation2d> headingSupplier) {
        m_targetHeadingSupplier = headingSupplier;

        setTargetHeadingEnabled(enabled);
    }

    public void setTargetHeadingEnabled(boolean enabled) {
        if (enabled && m_targetHeadingSupplier == null) {
            DriverStation.reportWarning("Target Heading enabled but m_targetHeadingSupplier == null.", true);
        }

        m_targetHeadingEnabled = enabled;
    }

    public void setCurrentMaxSpeed(Measure<Velocity<Distance>> speed) {
        m_currentMaxSpeed = speed;
    }

    // -- Commands --

    /**
     * Constructs a command that drives with given joystick inputs
     *
     * @param joyLeftXSupplier
     *            left joystick x
     * @param joyLeftYSupplier
     *            left joystick y
     * @param joyRightXSupplier
     *            right joystick x
     * @param fieldRelativeSupplier
     *            wether or not to drive field relative
     */
    public Command commandDriveWithJoystick(
        DoubleSupplier joyLeftXSupplier,
        DoubleSupplier joyLeftYSupplier,
        DoubleSupplier joyRightXSupplier,
        BooleanSupplier fieldRelativeSupplier
    ) {
        return this.run(() -> {
            double joyLeftX = m_driverControllerLeftXLimiter.calculate(
                MathUtil.applyDeadband(
                    joyLeftXSupplier.getAsDouble(),
                    Constants.DriveTeamConstants.kDriveJoystickDeadzone
                )
            );
            double joyLeftY = m_driverControllerLeftYLimiter.calculate(
                MathUtil.applyDeadband(
                    joyLeftYSupplier.getAsDouble(),
                    Constants.DriveTeamConstants.kDriveJoystickDeadzone
                )
            );
            double joyRightX = m_driverControllerRightXLimiter.calculate(
                MathUtil.applyDeadband(
                    joyRightXSupplier.getAsDouble(),
                    Constants.DriveTeamConstants.kDriveJoystickDeadzone
                )
            );

            // Early out if there is no driving to be done
            if (joyLeftX == 0.0 && joyLeftY == 0.0 && joyRightX == 0.0) {
                stop();
                return;
            }

            m_drivingState = SwerveDrivingState.DRIVING_JOYSTICK;

            boolean fieldRelative = fieldRelativeSupplier.getAsBoolean();

            double maxSpeedMetersPerSec = SwerveConstants.kDefaultSpeed.in(Units.MetersPerSecond);
            double maxAngularSpeedRadPerSec = SwerveConstants.kDefaultAngularSpeed.in(Units.RadiansPerSecond);

            double vx, vy, vrot;
            vx = joyLeftY * maxSpeedMetersPerSec;
            vy = joyLeftX * maxSpeedMetersPerSec;
            // if robot relative, invert vx and vy
            if (!fieldRelative) {
                vx = -vx;
                vy = -vy;
            }
            // If targeting heading, return 0 for desired vrot.
            // This isn't really required because 'drive' will ignore angular speed if target heading is enabled.
            vrot = !isTargetingHeading() ? -joyRightX * maxAngularSpeedRadPerSec : 0.0;

            drive(vx, vy, vrot, fieldRelative);
        }).withName("SwerveSubsystem::DriveWithJoystickCommand");
    }

    public Command commandFollowPathPlannerPath(PathPlannerPath path) {
        return AutoBuilder.followPath(path);
    }

    public Command commandSetPose(Pose2d pose) {
        return this.runOnce(() -> setPose(pose));
    }

    public Command commandSetTargetHeadingEnabled(boolean enabled) {
        // This command does not require this subsystem because it does not control the subsystem/it is ok for this
        // command and another command to control this subsystem.
        // It needs to be like this because auto paths may need to enable and disable heading targeting while driving.
        return Commands.runOnce(() -> setTargetHeadingEnabled(enabled));
    }

    public Command commandSetTargetHeadingEnabled(boolean enabled, Supplier<Rotation2d> targetHeadingSupplier) {
        // This command does not require this subsystem because it does not control the subsystem/it is ok for this
        // command and another command to control this subsystem.
        // It needs to be like this because auto paths may need to enable and disable heading targeting while driving.
        return Commands.runOnce(() -> setTargetHeadingEnabled(enabled, targetHeadingSupplier));
    }

    public Command commandSetMaxSpeed(Supplier<Measure<Velocity<Distance>>> speed) {
        return Commands.runOnce(() -> setCurrentMaxSpeed(speed.get()));
    }

    private void updatePoseEstimator() {
        m_poseEstimator.update(m_gyro.getYaw(), getModulePositions());

        VisionMeasurement[] measurements = StateMachine.VisionState.getMeasurements();

        if (measurements.length != 0) {
            for (VisionMeasurement measurement : measurements) {
                // Don't use this measurement for pose estimation if it is from a simulated
                // camera and kUseSimCameraForPoseEstimation is false
                if (measurement.isFromSimCamera() && !SwerveConstants.kUseSimCameraForPoseEstimation)
                    continue;

                m_poseEstimator.addVisionMeasurement(
                    measurement.getEstimatedPose(),
                    measurement.getTimestamp()
                );
            }
        }
    }

    private Rotation2d getDriveRotationOffset() {
        return Util.getAlliance() == Alliance.Red ? Rotation2d.fromRadians(0)
            : Rotation2d.fromRadians(Math.PI);
    }

    @Override
    public void periodic() {
        for (SwerveModule module : m_swerveModules) {
            module.periodic();
        }

        // If simulated gyro, update its yaw based off of angular chassis speed and
        // delta time (is this physically accurate? m_kinematics.toTwist2d() wouldn't
        // work)
        if (m_gyro.isSim()) {
            m_gyro.setYaw(
                Rotation2d.fromRadians(
                    MathUtil.applyDeadband(getRobotRelativeChassisSpeeds().omegaRadiansPerSecond, 0.45)
                        * Constants.kRobotLoopPeriodSecs
                ).plus(m_gyro.getYaw())
            );
        }

        updatePoseEstimator();

        if (DriverStation.isDisabled()) {
            stop();
        }

        ChassisSpeeds currentSpeeds = getRobotRelativeChassisSpeeds();

        if (
            currentSpeeds.vxMetersPerSecond >= 0.01
                && currentSpeeds.vyMetersPerSecond >= 0.01
                && currentSpeeds.omegaRadiansPerSecond >= 0.01
        ) {
            m_drivingState = SwerveDrivingState.MOVING_COASTING;
        }
        else {
            m_drivingState = SwerveDrivingState.STOPPED_COASTING;
        }
    }
}
