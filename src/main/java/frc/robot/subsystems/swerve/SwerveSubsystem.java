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
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.SwerveModuleConstants;
import frc.robot.util.InvalidSwerveModuleMotorConfigurationException;

import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends SubsystemBase {
    public static final Lock odometryLock = new ReentrantLock();

    private SwerveModule[] m_swerveModules;

    private Gyro m_gyro;

    private final SwerveDriveKinematics m_kinematics;
    private SwerveDrivePoseEstimator m_poseEstimator;

    private final SlewRateLimiter m_driverControllerLeftXLimiter;
    private final SlewRateLimiter m_driverControllerLeftYLimiter;
    private final SlewRateLimiter m_driverControllerRightXLimiter;

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
            m_swerveModules[0].getTranslation2d(), m_swerveModules[1].getTranslation2d(),
            m_swerveModules[2].getTranslation2d(), m_swerveModules[3].getTranslation2d()
        );

        m_poseEstimator = new SwerveDrivePoseEstimator(
            m_kinematics, Rotation2d.fromRotations(0), getModulePositions(), new Pose2d()
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
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            m_swerveModules[0].getPosition(),
            m_swerveModules[1].getPosition(),
            m_swerveModules[2].getPosition(),
            m_swerveModules[3].getPosition()
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
        ChassisSpeeds discrete = ChassisSpeeds.discretize(desiredSpeeds, Constants.kRobotLoopPeriod.in(Units.Seconds));
        ChassisSpeeds speeds = fieldRelative
            ? ChassisSpeeds.fromRobotRelativeSpeeds(discrete, getHeading().plus(getDriveRotationOffset()))
            : discrete;
        SwerveModuleState[] desiredStates = m_kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveModuleConstants.kMaxAttainableSpeed);

        SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
        for (int i = 0; i < desiredStates.length; i++) {
            optimizedStates[i] = m_swerveModules[i].setDesiredState(desiredStates[i]);
        }

        Logger.recordOutput("Swerve/DesiredStates", desiredStates);
        Logger.recordOutput("Swerve/OptimizedDesiredStates", optimizedStates);
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

    public Command driveWithJoystickCommand(
        DoubleSupplier joyLeftXSupplier, DoubleSupplier joyLeftYSupplier, DoubleSupplier joyRightXSupplier,
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

            boolean fieldRelative = fieldRelativeSupplier.getAsBoolean();

            double maxSpeedMetersPerSec = SwerveConstants.kDefaultSpeed.in(Units.MetersPerSecond);
            double maxAngularSpeedRadPerSec = SwerveConstants.kDefaultAngularSpeed.in(Units.RadiansPerSecond);

            double vx, vy, vrot;
            // swap vx and vy depending if robot or field relative
            if (fieldRelative) {
                vx = joyLeftY * maxSpeedMetersPerSec;
                vy = joyLeftX * maxSpeedMetersPerSec;
            }
            else {
                vx = joyLeftX * maxSpeedMetersPerSec;
                vy = joyLeftY * maxSpeedMetersPerSec;
            }

            vrot = joyRightX * maxAngularSpeedRadPerSec;

            drive(vx, vy, vrot, fieldRelative);
        }).withName("DriveWithJoystickCommand");
    }

    private void updateOdometry() {
        m_poseEstimator.update(m_gyro.getYaw(), getModulePositions());
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

        updateOdometry();

        // if (DriverStation.isDisabled()) {
        // stop();
        // }

        Logger.recordOutput("Swerve/CurrentModuleStates", getModuleStates());
        Logger.recordOutput("Swerve/EstimatedPose", getPose());
    }
}
