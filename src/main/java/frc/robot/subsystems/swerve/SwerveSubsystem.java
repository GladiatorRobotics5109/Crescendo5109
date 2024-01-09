package frc.robot.subsystems.swerve;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathHolonomic;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Util.EstimatedPose;

public class SwerveSubsystem extends SubsystemBase {
    // swerve modules
    private final SwerveModule m_moduleFL;
    private final SwerveModule m_moduleFR;
    private final SwerveModule m_moduleBL;
    private final SwerveModule m_moduleBR;

    // speeds
    private double m_maxSpeed;
    private final double m_defaultSpeed;
    private double m_maxAngularSpeed;

    private final AHRS m_navX;
    
    // pose stuff
    private final SwerveDriveKinematics m_kinematics;
    private final SwerveDrivePoseEstimator m_poseEstimator;

    private final PPHolonomicDriveController m_holonomicDriveController;

    private final Timer m_trajectoryDriveTimer;
    private final Boolean m_drivingWithTrajectory;
    // private final HolonomicDriveController m_holonomicDriveController;
    
    public SwerveSubsystem() {
        // can ids used for Wobbles, may be different depending on bot
        m_moduleFL = new SwerveModuleNeoTurnNeoDrive(Constants.SwerveConstants.kModulePosFrontLeft, "frontLeft", 0, 15, 14);
        m_moduleFR = new SwerveModuleNeoTurnNeoDrive(Constants.SwerveConstants.kModulePosFrontRight, "frontRight", 1, 12, 13);
        m_moduleBL = new SwerveModuleNeoTurnNeoDrive(Constants.SwerveConstants.kModulePosBackLeft, "backLeft", 2, 18, 19);
        m_moduleBR = new SwerveModuleNeoTurnNeoDrive(Constants.SwerveConstants.kModulePosBackRight, "backRight", 3, 16, 17);

        // set speeds
        m_maxSpeed = SwerveConstants.kDefaultSpeed;
        m_defaultSpeed = SwerveConstants.kDefaultSpeed;
        m_maxAngularSpeed = SwerveConstants.kMaxAngularSpeed;
        
        m_navX = SwerveConstants.kNavX;
        //TODO: m_navX.reset(); <-- Test this?????
        m_navX.setAngleAdjustment(90);
        
        // initialize kinematics
        m_kinematics = new SwerveDriveKinematics(
            m_moduleFL.getPoseRelative(), 
            m_moduleFR.getPoseRelative(),
            m_moduleBL.getPoseRelative(), 
            m_moduleBR.getPoseRelative()
        );
        
        Optional<EstimatedPose> startingPose = VisionManager.getPose();

        m_poseEstimator = new SwerveDrivePoseEstimator(
            m_kinematics,
            getHeading(), 
            getModulePositions(),
            startingPose.isEmpty() ? new Pose2d() : startingPose.get().getPose()
        );
        
        // initialize holonomic drive controller
        // m_holonomicDriveController = new HolonomicDriveController(
        //     Constants.SwerveConstants.kPidControllerHolonomicX,
        //     Constants.SwerveConstants.kPidControllerHolonomicY,
        //     Constants.SwerveConstants.kPidControllerHolonomicRot
        // );
        m_holonomicDriveController = new PPHolonomicDriveController(
            Constants.SwerveConstants.kTranslationConstants,
            Constants.SwerveConstants.kRotationConstants,
            0.2,
            Constants.SwerveConstants.kDefaultSpeed,
            Constants.SwerveConstants.kModulePosFrontLeft.getDistance(new Translation2d())
        );

        m_trajectoryDriveTimer = new Timer();
        m_drivingWithTrajectory = false;
    }

    /** drive with desired x/y/rot velocities */
    public void drive(double vx, double vy, double rot, boolean fieldRelative) {
        // TODO: change new Rotation2d... to Rotation2d.fromDegrees(-m_navX.getAngle());?
        // Rotation2d navXVal = new Rotation2d((-m_navX.getAngle() % 360) * Math.PI / 180);
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rot, getHeading()) : new ChassisSpeeds(vx, vy, rot));
        // apply max speeds
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, m_maxSpeed);

        m_moduleFL.setDesiredState(swerveModuleStates[0]);
        m_moduleFR.setDesiredState(swerveModuleStates[1]);
        m_moduleBL.setDesiredState(swerveModuleStates[2]);
        m_moduleBR.setDesiredState(swerveModuleStates[3]);
    }

    /** drive with desired chassis speeds */
    public void drive(ChassisSpeeds desiredSpeeds, boolean fieldRelative) {
        // TODO: change new Rotation2d... to Rotation2d.fromDegrees(-m_navX.getAngle());?
        Rotation2d navXVal = new Rotation2d((m_navX.getAngle() % 360) * Math.PI / 180);
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
            fieldRelative ? 
            ChassisSpeeds.fromFieldRelativeSpeeds(
                desiredSpeeds.vxMetersPerSecond, 
                desiredSpeeds.vyMetersPerSecond, 
                desiredSpeeds.omegaRadiansPerSecond, 
                navXVal
            ) : 
            desiredSpeeds
        );

        // apply max speeds
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, m_maxSpeed);
        
        m_moduleFL.setDesiredState(swerveModuleStates[0]);
        m_moduleFR.setDesiredState(swerveModuleStates[1]);
        m_moduleBL.setDesiredState(swerveModuleStates[2]);
        m_moduleBR.setDesiredState(swerveModuleStates[3]);
    }

    public void stop() {
        drive(0, 0, 0, Constants.SwerveConstants.kFieldRelative);
    }

    public void setMaxSpeed(double speed) {
        m_maxSpeed = speed;
    }

    /** 
     * Brake and X the wheels to stay still
    */
    public void brakeAndX() {
        stop();
        
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, m_maxSpeed);
        swerveModuleStates[0].angle = Rotation2d.fromRadians(Math.PI / 4);
        swerveModuleStates[1].angle = Rotation2d.fromRadians(-Math.PI / 4);
        swerveModuleStates[2].angle = Rotation2d.fromRadians(-Math.PI / 4);
        swerveModuleStates[3].angle = Rotation2d.fromRadians(Math.PI / 4);

        m_moduleFL.setDesiredState(swerveModuleStates[0]);
        m_moduleFR.setDesiredState(swerveModuleStates[1]);
        m_moduleBL.setDesiredState(swerveModuleStates[2]);
        m_moduleBR.setDesiredState(swerveModuleStates[3]);

        brakeAll();
    }

    /** Brake on all motors on all swerve modules */
    private void brakeAll() {
        m_moduleFL.brakeAll();
        m_moduleFL.brakeAll();
        m_moduleFL.brakeAll();
        m_moduleFL.brakeAll();
    }
    
    /** Coast on all motors on all swerve modules */
    public void coastAll() {
        m_moduleFL.coastAll();
        m_moduleFR.coastAll();
        m_moduleBL.coastAll();
        m_moduleBR.coastAll();
    }
    
    public void resetTurnEncoders() {
        m_moduleFL.resetTurnEncoder();
        m_moduleFR.resetTurnEncoder();
        m_moduleBL.resetTurnEncoder();
        m_moduleFR.resetTurnEncoder();
    }
    
    /**
     * @return a command object that drives with given joystick inputs
     */
    public Command getDriveWithJoystickCommand(
        DoubleSupplier joyLeftX, 
        DoubleSupplier joyLeftY, 
        DoubleSupplier joyRightX, 
        BooleanSupplier fieldRelative) {
        return this.run(() -> {
            updatePose();

            // get joystick axises
            double vx = MathUtil.applyDeadband(joyLeftX.getAsDouble(), Constants.DriveTeamConstants.kJoystickDeadzone);
            double vy = MathUtil.applyDeadband(joyLeftY.getAsDouble(), Constants.DriveTeamConstants.kJoystickDeadzone);
            double rot = MathUtil.applyDeadband(joyRightX.getAsDouble(), Constants.DriveTeamConstants.kJoystickDeadzone);

            // apply max speeds
            vx *= m_maxSpeed;
            vy *= m_maxSpeed;
            rot *= m_maxAngularSpeed;

            drive(vx, vy, rot, fieldRelative.getAsBoolean());
        }).withName("DriveWithJoystickCommand");
    }

    /** drives with a given path */
    public Command getDriveWithPathCommand(PathPlannerPath path) {
        return new FollowPathHolonomic(
            path,
            () -> getPose(), 
            () -> getSpeeds(),
            (chassisSpeeds) -> {
                SwerveDriveKinematics.desaturateWheelSpeeds(getStates(), m_maxSpeed);

                drive(chassisSpeeds, Constants.SwerveConstants.kFieldRelative);
            },
            Constants.AutoConstants.kHolonomicPathConfig, 
            () -> Constants.AutoConstants.kAutoMirrorPath,
            this
        );
    }
    
    /**
     * 
     * @return A Command object that aligns
     */
    public Command getAlignWheelCommand() {
        return this.runOnce(() -> {
            SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, m_maxSpeed);
            swerveModuleStates[0].angle = Rotation2d.fromRadians(0);
            swerveModuleStates[1].angle = Rotation2d.fromRadians(0);
            swerveModuleStates[2].angle = Rotation2d.fromRadians(0);
            swerveModuleStates[3].angle = Rotation2d.fromRadians(0);


            m_moduleFL.setDesiredState(swerveModuleStates[0]);
            m_moduleFR.setDesiredState(swerveModuleStates[1]);
            m_moduleBL.setDesiredState(swerveModuleStates[2]);
            m_moduleBR.setDesiredState(swerveModuleStates[3]);
        });
    }

    public Command getBrakeAndXCommand() {
        return this.runOnce(() -> {
            brakeAndX();
        });
    }

    public Command getCoastAllCommand() {
        return this.runOnce(() -> {
            coastAll();
        });
    }

    public Command getSuperSpeedCommand(DoubleSupplier scalar) {
        return this.runOnce(() -> {
            setMaxSpeed((5.0 * scalar.getAsDouble()) + m_defaultSpeed);
        });
    }

    public Command getSetSlowModeCommand(BooleanSupplier isSlowMode) {
        return this.runOnce(() -> {
            setMaxSpeed(isSlowMode.getAsBoolean() ? Constants.SwerveConstants.kSlowModeSpeed : m_defaultSpeed);
        });
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            m_moduleFL.getModulePose(),
            m_moduleFR.getModulePose(),
            m_moduleBL.getModulePose(),
            m_moduleBR.getModulePose(),
        };
    }

    private void updatePose() {
        m_poseEstimator.updateWithTime(
            Timer.getFPGATimestamp(), 
            getHeading(), 
            getModulePositions()
        );
        
        Optional<EstimatedPose> visionPose = VisionManager.getPose();
        
        // if vision result
        if (!visionPose.isEmpty()) { 
            m_poseEstimator.addVisionMeasurement(
                visionPose.get().getPose(), 
                visionPose.get().getTimestampSeconds()
            );
        }
    }

    private Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }
    
    public Rotation2d getHeading() {
        return m_navX.getRotation2d();
    }

    public SwerveModuleState[] getStates() {
        return new SwerveModuleState[] {
            m_moduleFL.getState(),
            m_moduleFR.getState(),
            m_moduleBL.getState(),
            m_moduleBR.getState()
        };
    }

    public ChassisSpeeds getSpeeds() {
        return m_kinematics.toChassisSpeeds(getStates());
    }
}
