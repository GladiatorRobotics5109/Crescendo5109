package frc.robot.subsystems.swerve;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.SwerveConstants;
import frc.robot.vision.EstimatedVisionPosition;
import frc.robot.vision.VisionManager;

public class SwerveSubsystem extends SubsystemBase {
    // swerve modules
    private final SwerveModuleNeoTurnNeoDrive m_moduleFL;
    private final SwerveModuleNeoTurnNeoDrive m_moduleFR;
    private final SwerveModuleNeoTurnNeoDrive m_moduleBL;
    private final SwerveModuleNeoTurnNeoDrive m_moduleBR;
    
    private final double m_defaultSpeed;
    private double m_currentSpeed;
    private double m_maxAngularSpeed;

    private final SwerveDriveKinematics m_kinematics;

    private final VisionManager m_vision;

    private final SwerveDrivePoseEstimator m_poseEstimator;
    
    private final AHRS m_navX;
    
    public SwerveSubsystem() {
        // TODO: select right CAN ids for motors
        m_moduleFL = new SwerveModuleNeoTurnNeoDrive(Constants.SwerveConstants.kModulePosFrontLeft, "frontLeft", 0, 15, 14);
        m_moduleFR = new SwerveModuleNeoTurnNeoDrive(Constants.SwerveConstants.kModulePosFrontRight, "frontRight", 1, 12, 13);;
        m_moduleBL = new SwerveModuleNeoTurnNeoDrive(Constants.SwerveConstants.kModulePosBackLeft, "backLeft", 2, 18, 19);;
        m_moduleBR = new SwerveModuleNeoTurnNeoDrive(Constants.SwerveConstants.kModulePosBackRight, "backRight", 3, 16, 17);;
    
        m_navX = SwerveConstants.kNavX;
        //m_navX.reset(); <-- Test this?????
        m_navX.setAngleAdjustment(90);
        
        m_kinematics = new SwerveDriveKinematics(
            m_moduleFL.getPos(), 
            m_moduleFR.getPos(),
            m_moduleBL.getPos(), 
            m_moduleBR.getPos()
        );

        m_vision = new VisionManager(Constants.VisionConstants.kVisionSources);

        Optional<EstimatedVisionPosition> startingPose = m_vision.getPose();

        m_poseEstimator = new SwerveDrivePoseEstimator(
            m_kinematics,
            getHeading(),
            getPositions(),
            startingPose.isEmpty() ? new Pose2d() : startingPose.get().getPose2d()
        );

        m_defaultSpeed = SwerveConstants.kMaxSpeed;
        m_currentSpeed = m_defaultSpeed;
        m_maxAngularSpeed = SwerveConstants.kMaxAngularSpeed;
    }

    /** drive with desired x/y/rot velocities */
    public void drive(double vx, double vy, double rot, boolean fieldRelative) {
        Rotation2d navXVal = new Rotation2d((-m_navX.getAngle() % 360) * Math.PI / 180);
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rot, navXVal) : new ChassisSpeeds(vx, vy, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, m_currentSpeed);

        m_moduleFL.setDesiredState(swerveModuleStates[0]);
        m_moduleFR.setDesiredState(swerveModuleStates[1]);
        m_moduleBL.setDesiredState(swerveModuleStates[2]);
        m_moduleBR.setDesiredState(swerveModuleStates[3]);
    }

    /** drive with desired chassis speeds */
    public void drive(ChassisSpeeds desiredSpeeds, boolean fieldRelative) {
        drive(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond, desiredSpeeds.omegaRadiansPerSecond, fieldRelative);
    }

    public void setMaxSpeed(double speed) {
        m_currentSpeed = speed;
    }

    /** Brake and X the wheels to stay still */
    public void brakeAndX() {
        SwerveModuleState flBr = new SwerveModuleState();
        flBr.speedMetersPerSecond = 0.0;
        flBr.angle = Rotation2d.fromRadians(-Math.PI / 4);

        m_moduleFL.setDesiredState(flBr);
        m_moduleBR.setDesiredState(flBr);

        SwerveModuleState frBl = new SwerveModuleState();
        frBl.speedMetersPerSecond = 0.0;
        frBl.angle = Rotation2d.fromRadians(Math.PI / 4);

        m_moduleFR.setDesiredState(frBl);
        m_moduleBL.setDesiredState(frBl);

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
    public void coast() {
        m_moduleFL.coastAll();
        m_moduleFR.coastAll();
        m_moduleBL.coastAll();
        m_moduleBR.coastAll();
    }
    
    /**
     * @return a command object that drives with given joystick inputs
     */
    public Command getDriveWithJoystickCommand(
        DoubleSupplier joyLeftX, 
        DoubleSupplier joyLeftY, 
        DoubleSupplier joyRightX,
        DoubleSupplier joyLeftTrigger,
        DoubleSupplier joyRightTrigger,
        BooleanSupplier fieldRelative) {
        return run(() -> {
            // get joystick axises
            double vx = MathUtil.applyDeadband(joyLeftX.getAsDouble(), Constants.kJoystickDeadzone);
            double vy = MathUtil.applyDeadband(joyLeftY.getAsDouble(), Constants.kJoystickDeadzone);
            double rot = MathUtil.applyDeadband(joyRightX.getAsDouble(), Constants.kJoystickDeadzone);

            double newSpeed = ((5 * joyLeftTrigger.getAsDouble()) + m_defaultSpeed) + (-8 * joyRightTrigger.getAsDouble());

            setMaxSpeed(newSpeed);

            // apply max speeds
            vx *= m_currentSpeed;
            vy *= m_currentSpeed;
            rot *= m_maxAngularSpeed;

            drive(vx, vy, rot, fieldRelative.getAsBoolean());
        }).withName("DriveWithJoystickCommand");
    }

    public Command getAlignWheelCommand() {
        return this.run(() -> {
            SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));
            SwerveDriveKinematics.desaturateWheelSpeeds(states, m_currentSpeed);
            
            m_moduleBL.getState().angle.getRotations();
            
            states[0].angle = Rotation2d.fromRadians(0);
            states[1].angle = Rotation2d.fromRadians(0);
            states[2].angle = Rotation2d.fromRadians(0);
            states[3].angle = Rotation2d.fromRadians(0);

            m_moduleFL.setDesiredState(states[0], false);
            m_moduleFR.setDesiredState(states[1], false);
            m_moduleBL.setDesiredState(states[2], false);
            m_moduleBR.setDesiredState(states[3], false);
        }).withName("alignWheelCommand");
    }

    public Command getFollowPathCommand(PathPlannerPath path) {
        return new FollowPathHolonomic(
            path,
            this::getPose,
            this::getSpeeds,
            (ChassisSpeeds chassisSpeeds) -> drive(chassisSpeeds, false),
            SwerveConstants.AutonConstants.kTranslationPID,
            SwerveConstants.AutonConstants.kRotationPID,
            SwerveConstants.AutonConstants.kMaxSpeed,
            SwerveConstants.kDriveBaseRadius,
            Robot.kDefaultPeriod,
            SwerveConstants.AutonConstants.kReplanningConfig,
            () -> SwerveConstants.AutonConstants.kAutoMirror,
            this
        );
    }

    public Command getBrakeAndXCommand() {
        return this.runOnce(() -> brakeAndX()).withName("brakeAndXCommand");
    }

    public Command getSuperSpeedCommand(DoubleSupplier scalar) {
        return this.runOnce(() -> setMaxSpeed((5 * scalar.getAsDouble()) + m_defaultSpeed));
    }

    public Command getSuperSlowCommand(DoubleSupplier scalar) {
        return this.runOnce(() -> setMaxSpeed(-5 * scalar.getAsDouble() + m_defaultSpeed));
    }

    private Rotation2d getHeading() {
        return m_navX.getRotation2d();
    }

    private SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
            m_moduleFL.getModulePosition(),
            m_moduleFR.getModulePosition(),
            m_moduleBL.getModulePosition(),
            m_moduleBR.getModulePosition()
        };
    }

    public void resetEncoders() {
        m_moduleFL.resetEncoders();
        m_moduleFR.resetEncoders();
        m_moduleBL.resetEncoders();
        m_moduleBR.resetEncoders(); 
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

    private void updatePose() {
        m_poseEstimator.update(getHeading(), getPositions());

        Optional<EstimatedVisionPosition> poses = m_vision.getPose();

        if (poses.isEmpty()) return;

        for (EstimatedRobotPose pose : poses.get().getEstimatedRobotPoses()) {
            m_poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
        }
    }

    private Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("vx", getSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("vy", getSpeeds().vyMetersPerSecond);
        SmartDashboard.putNumber("rot", getSpeeds().omegaRadiansPerSecond);

        updatePose();

        Pose2d pose = getPose();
        // System.out.println("Pose (" + pose.getX() + ", " + pose.getY() + ")");
        SmartDashboard.putNumber("posx", pose.getX());
        SmartDashboard.putNumber("posy", pose.getY());

        SmartDashboard.putNumber("ModFL DrivePos", m_moduleFL.getDrivePosition());
        SmartDashboard.putNumber("ModFR DrivePos", m_moduleFR.getDrivePosition());
        SmartDashboard.putNumber("ModBL DrivePos", m_moduleBL.getDrivePosition());
        SmartDashboard.putNumber("ModBR DrivePos", m_moduleBR.getDrivePosition());

        SmartDashboard.putNumber("ModFL DriveVelocity", m_moduleFL.getDriveVelocity());
        SmartDashboard.putNumber("ModFR DriveVelocity", m_moduleFR.getDriveVelocity());
        SmartDashboard.putNumber("ModBL DriveVelocity", m_moduleBL.getDriveVelocity());
        SmartDashboard.putNumber("ModBR DriveVelocity", m_moduleBR.getDriveVelocity());

        SmartDashboard.putNumber("ModFL TurnPos", m_moduleFL.getTurnPosition());
        SmartDashboard.putNumber("ModFR TurnPos", m_moduleFR.getTurnPosition());
        SmartDashboard.putNumber("ModBL TurnPos", m_moduleBL.getTurnPosition());
        SmartDashboard.putNumber("ModBR TurnPos", m_moduleBR.getTurnPosition());

    }
}
