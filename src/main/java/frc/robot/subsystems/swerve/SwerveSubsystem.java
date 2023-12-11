package frc.robot.subsystems.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {
    // swerve modules
    private final SwerveModule m_moduleFL;
    private final SwerveModule m_moduleFR;
    private final SwerveModule m_moduleBL;
    private final SwerveModule m_moduleBR;

    private double m_maxSpeed;
    private final double m_defaultSpeed;
    private double m_maxAngularSpeed;

    private final SwerveDriveKinematics m_kinematics;
    private final SwerveDriveOdometry m_odometry;

    private final AHRS m_navX;

    private final HolonomicDriveController m_holonomicDriveController;
    
    public SwerveSubsystem() {
        // can ids used for Wobbles, may be different depending on bot
        m_moduleFL = new SwerveModuleNeoTurnNeoDrive(Constants.SwerveConstants.kModulePosFrontLeft, "frontLeft", 0, 15, 14);
        m_moduleFR = new SwerveModuleNeoTurnNeoDrive(Constants.SwerveConstants.kModulePosFrontRight, "frontRight", 1, 12, 13);
        m_moduleBL = new SwerveModuleNeoTurnNeoDrive(Constants.SwerveConstants.kModulePosBackLeft, "backLeft", 2, 18, 19);
        m_moduleBR = new SwerveModuleNeoTurnNeoDrive(Constants.SwerveConstants.kModulePosBackRight, "backRight", 3, 16, 17);
        
        m_navX = SwerveConstants.kNavX;
        //m_navX.reset(); <-- Test this?????
        m_navX.setAngleAdjustment(90);
        
        m_kinematics = new SwerveDriveKinematics(
            m_moduleFL.getPos(), 
            m_moduleFR.getPos(),
            m_moduleBL.getPos(), 
            m_moduleBR.getPos());
        
        m_odometry = new SwerveDriveOdometry(
            m_kinematics, 
            Rotation2d.fromDegrees(m_navX.getAngle()), 
            getPositions(),
            new Pose2d(
                new Translation2d(0, 0), 
                Rotation2d.fromDegrees(m_navX.getAngle())
            ));

        m_maxSpeed = SwerveConstants.kDefaultSpeed;
        m_defaultSpeed = SwerveConstants.kDefaultSpeed;
        m_maxAngularSpeed = SwerveConstants.kMaxAngularSpeed;

        m_holonomicDriveController = new HolonomicDriveController(
            Constants.SwerveConstants.kPidControllerHolonomicX,
            Constants.SwerveConstants.kPidControllerHolonomicY,
            Constants.SwerveConstants.kPidControllerHolonomicRot);
    }


    /** drive with desired x/y/rot velocities */
    public void drive(double vx, double vy, double rot, boolean fieldRelative) {
        Rotation2d navXVal = new Rotation2d((-m_navX.getAngle() % 360) * Math.PI / 180);
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rot, navXVal) : new ChassisSpeeds(vx, vy, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, m_maxSpeed);

        m_moduleFL.setDesiredState(swerveModuleStates[0]);
        m_moduleFR.setDesiredState(swerveModuleStates[1]);
        m_moduleBL.setDesiredState(swerveModuleStates[2]);
        m_moduleBR.setDesiredState(swerveModuleStates[3]);
    }

    /** drive with desired chassis speeds */
    public void drive(ChassisSpeeds desiredSpeeds, boolean fieldRelative) {
        Rotation2d navXVal = new Rotation2d((m_navX.getAngle() % 360) * Math.PI / 180);
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond, desiredSpeeds.omegaRadiansPerSecond, navXVal) : desiredSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, m_maxSpeed);
        
        m_moduleFL.setDesiredState(swerveModuleStates[0]);
        m_moduleFR.setDesiredState(swerveModuleStates[1]);
        m_moduleBL.setDesiredState(swerveModuleStates[2]);
        m_moduleBR.setDesiredState(swerveModuleStates[3]);
    }

    public void stop() {
        drive(0, 0, 0, true);
    }

    public void setMaxSpeed(double speed) {
        m_maxSpeed = speed;
    }

    /** Brake and X the wheels to stay still */
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

    public void resetEncoders() {
        m_moduleFL.resetTurnEncoder();
        m_moduleFR.resetTurnEncoder();
        m_moduleBL.resetTurnEncoder();
        m_moduleFR.resetTurnEncoder();
    }

    private SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
            m_moduleFL.getState(),
            m_moduleFR.getState(),
            m_moduleBL.getState(),
            m_moduleBR.getState(),
        };
    }

    private Pose2d getPose() {
        return m_odometry.update(Rotation2d.fromDegrees(m_navX.getAngle()), getPositions());
    }
    
    /**
     * @return a command object that drives with given joystick inputs
     */
    public Command getDriveWithJoystickCommand(
        DoubleSupplier joyLeftX, 
        DoubleSupplier joyLeftY, 
        DoubleSupplier joyRightX, 
        BooleanSupplier fieldRelative) {
        return run(() -> {
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
    
    public Command getDriveWithTrajectoryCommand(
        DoubleSupplier time,
        Trajectory trajectory) {
        return this.run(() -> {
            Trajectory.State desiredState = trajectory.sample(time.getAsDouble());
            Pose2d pos = getPose();

            ChassisSpeeds chassisSpeeds = m_holonomicDriveController.calculate(
            pos, desiredState, Rotation2d.fromRotations(0));

            drive(chassisSpeeds, true);
        }).withName("HolonomicDriveCommand");
    }
    
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
}
