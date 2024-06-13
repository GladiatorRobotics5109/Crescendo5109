package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.SwerveModuleConstants;
import frc.robot.stateMachine.StateMachine;
import frc.robot.stateMachine.StateMachine.SwerveState.SwerveDrivingState;
import frc.robot.subsystems.swerve.gyro.Gyro;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIONavX;
import frc.robot.subsystems.swerve.gyro.GyroIOSim;
import frc.robot.subsystems.swerve.swerveModule.SwerveModule;
import frc.robot.subsystems.vision.VisionMeasurement;
import frc.robot.util.InvalidSwerveModuleMotorConfigurationException;

import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

public class SwerveSubsystem extends SubsystemBase {
    public static final Lock odometryLock = new ReentrantLock();

    private SwerveModule[] m_swerveModules;

    private Gyro m_gyro;

    private final SwerveDriveKinematics m_kinematics;
    private final SwerveDrivePoseEstimator m_poseEstimator;

    private final SlewRateLimiter m_driverControllerLeftXLimiter;
    private final SlewRateLimiter m_driverControllerLeftYLimiter;
    private final SlewRateLimiter m_driverControllerRightXLimiter;

    private SwerveDrivingState m_drivingState;

    private final PIDController m_autonXPID;
    private final PIDController m_autonYPID;
    private final PIDController m_autonRotPID;

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

                    m_gyro = new Gyro(new GyroIONavX());

                    break;
                case SIM:
                    m_swerveModules = new SwerveModule[] {
                        new SwerveModule(SwerveModuleConstants.kFrontLeftSimModuleConstants),
                        new SwerveModule(SwerveModuleConstants.kFrontRightSimModuleConstants),
                        new SwerveModule(SwerveModuleConstants.kBackLeftSimModuleConstants),
                        new SwerveModule(SwerveModuleConstants.kBackRightSimModuleConstants)
                    };

                    m_gyro = new Gyro(new GyroIOSim());

                    break;
                default:
                    m_swerveModules = new SwerveModule[] {
                        new SwerveModule(SwerveModuleConstants.kReplayModuleConstants),
                        new SwerveModule(SwerveModuleConstants.kReplayModuleConstants),
                        new SwerveModule(SwerveModuleConstants.kReplayModuleConstants),
                        new SwerveModule(SwerveModuleConstants.kReplayModuleConstants)
                    };

                    m_gyro = new Gyro(new GyroIO() {});

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

        m_autonXPID = SwerveConstants.kAutonXPID.get();
        m_autonYPID = SwerveConstants.kAutonYPID.get();
        m_autonRotPID = SwerveConstants.kAutonRotPID.get();
    }

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

    public void drive(double vx, double vy, double vrot, boolean fieldRelative) {
        drive(new ChassisSpeeds(vx, vy, vrot), fieldRelative);
    }

    public void drive(ChassisSpeeds desiredSpeeds, boolean fieldRelative) {
        ChassisSpeeds discreteSpeeds = ChassisSpeeds
            .discretize(desiredSpeeds, Constants.kRobotLoopPeriod.in(Units.Seconds));
        ChassisSpeeds speeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(discreteSpeeds, getHeading().plus(getDriveRotationOffset()))
            : discreteSpeeds;
        SwerveModuleState[] desiredStates = m_kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveModuleConstants.kMaxAttainableSpeed);

        SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
        for (int i = 0; i < desiredStates.length; i++) {
            optimizedStates[i] = m_swerveModules[i].setDesiredState(desiredStates[i]);
        }

        // Change state if the desired state isn't already set to stopping
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

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            m_swerveModules[0].getState(),
            m_swerveModules[1].getState(),
            m_swerveModules[2].getState(),
            m_swerveModules[3].getState(),
        };
    }

    public void setPose(Pose2d pose) {
        m_poseEstimator.resetPosition(m_gyro.getYaw(), getModulePositions(), pose);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     *
     * @return what the swerve subsystem is currently doing
     */
    public SwerveDrivingState getDrivingState() {
        return m_drivingState;
    }

    public boolean isDriving() {
        SwerveDrivingState drivingState = getDrivingState();

        return drivingState == SwerveDrivingState.DRIVING_FOLLOWING_PATH
            || drivingState == SwerveDrivingState.DRIVING_JOYSTICK
            || drivingState == SwerveDrivingState.DRIVING_UNKNOWN;
    }

    public boolean isMoving() {
        SwerveDrivingState drivingState = getDrivingState();

        return drivingState == SwerveDrivingState.MOVING_BRAKING
            || drivingState == SwerveDrivingState.MOVING_COASTING
            || isDriving();
    }

    public boolean isStopped() {
        SwerveDrivingState drivingState = getDrivingState();

        return drivingState == SwerveDrivingState.STOPPED_BRAKING
            || drivingState == SwerveDrivingState.STOPPED_COASTING;
    }

    public Command driveWithJoystickCommand(
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
            if (!fieldRelative) {
                vx = -vx;
                vy = -vy;
            }
            vrot = -joyRightX * maxAngularSpeedRadPerSec;

            drive(vx, vy, vrot, fieldRelative);
        }).withName("DriveWithJoystickCommand");
    }

    public Command followTrajectoryCommand(ChoreoTrajectory traj) {
        return Choreo.choreoSwerveCommand(
            traj,
            this::getPose,
            m_autonXPID,
            m_autonYPID,
            m_autonRotPID,
            (ChassisSpeeds speeds) -> drive(speeds, false),
            () -> false,
            this
        );
    }

    public Command setPoseCommand(Pose2d pose) {
        return this.runOnce(() -> setPose(pose));
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
        Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
        Alliance alliance;
        if (optionalAlliance.isPresent()) {
            alliance = optionalAlliance.get();
        }
        else {
            alliance = Constants.kDefaultAlliance;
        }

        return alliance == Alliance.Red ? Rotation2d.fromRadians(0) : Rotation2d.fromRadians(Math.PI);
    }

    @Override
    public void periodic() {
        for (SwerveModule module : m_swerveModules) {
            module.periodic();
        }

        m_gyro.periodic();

        // If simulated gyro, update its yaw based off of angular chassis speed and
        // delta time (is this physically accurate? m_kinematics.toTwist2d() wouldn't
        // work)
        if (m_gyro.isSim()) {
            m_gyro.setYaw(
                Rotation2d.fromRadians(
                    MathUtil.applyDeadband(getChassisSpeeds().omegaRadiansPerSecond, 0.45)
                        * Constants.kRobotLoopPeriodSecs
                ).plus(m_gyro.getYaw())
            );
        }

        updatePoseEstimator();

        if (DriverStation.isDisabled()) {
            stop();
        }

        ChassisSpeeds currentSpeeds = getChassisSpeeds();

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
