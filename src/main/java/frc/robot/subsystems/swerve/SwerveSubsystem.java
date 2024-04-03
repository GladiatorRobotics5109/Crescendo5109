package frc.robot.subsystems.swerve;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.robot.stateMachine.StateMachine;
import frc.robot.stateMachine.SwerveState;
import frc.robot.stateMachine.SwerveState.SwerveStateEnum;
import frc.robot.util.logging.LoggablePose2d;
import frc.robot.util.logging.LoggableSwerveModuleStates;

import org.photonvision.EstimatedRobotPose;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.Constants;
import frc.robot.Robot;
import frc.robot.util.Constants.SwerveConstants;
import frc.robot.util.logging.LoggableBoolean;
import frc.robot.util.logging.LoggableDouble;
import frc.robot.util.logging.Logger;
import frc.robot.vision.EstimatedRobotPoses;
import frc.robot.vision.VisionManager;

public class SwerveSubsystem extends SubsystemBase {

    public enum ModuleOrientation {
        CIRCLE,
        BRAKE_X,
        ALIGNED
    }
    // swerve modules
    private final SwerveModuleNeoTurnKrakenDrive m_moduleFL;
    private final SwerveModuleNeoTurnKrakenDrive m_moduleFR;
    private final SwerveModuleNeoTurnKrakenDrive m_moduleBL;
    private final SwerveModuleNeoTurnKrakenDrive m_moduleBR;
    

    private final SwerveDriveKinematics m_kinematics;

    private final VisionManager m_vision;

    private final SwerveDrivePoseEstimator m_poseEstimator;
    
    private final AHRS m_navX;
    
    private final PIDController m_yawPIDController;
    private boolean m_autoAiming;

    private final LoggableDouble m_autoAimAngleLog;
    private final LoggableDouble m_autoAimPIDSetpointLog;
    private final LoggableBoolean m_autoAimStateLog;
    private final LoggablePose2d m_poseLog;

    private final LoggableSwerveModuleStates m_moduleStatesLog;
    private final LoggableSwerveModuleStates m_moduleDesiredStatesLog;
    
    private final SwerveState m_state;

    private final Trigger m_reachedAutoAimSetpointTrigger;

    public SwerveSubsystem() {
        m_state = StateMachine.getSwerveState();

        // TODO: select right CAN ids for motors
        m_moduleFL = new SwerveModuleNeoTurnKrakenDrive(Constants.SwerveConstants.kModulePosFrontLeft, "frontLeft", 0, 2, 5, Constants.SwerveConstants.kModuleEncoderOffsetFrontLeft);
        m_moduleFR = new SwerveModuleNeoTurnKrakenDrive(Constants.SwerveConstants.kModulePosFrontRight, "frontRight", 1, 4, 40, Constants.SwerveConstants.kModuleEncoderOffsetFrontRight);
        m_moduleBL = new SwerveModuleNeoTurnKrakenDrive(Constants.SwerveConstants.kModulePosBackLeft, "backLeft", 2, 1, 22, Constants.SwerveConstants.kModuleEncoderOffsetBackLeft);
        m_moduleBR = new SwerveModuleNeoTurnKrakenDrive(Constants.SwerveConstants.kModulePosBackRight, "backRight", 3, 3, 30, Constants.SwerveConstants.kModuleEncoderOffsetBackRight);
    
        m_navX = SwerveConstants.kNavX;
        m_navX.reset();
        m_navX.setAngleAdjustment(90);
        
        m_kinematics = new SwerveDriveKinematics(
            m_moduleFL.getPos(), 
            m_moduleFR.getPos(),
            m_moduleBL.getPos(), 
            m_moduleBR.getPos()
        );

        m_vision = new VisionManager(Constants.VisionConstants.kVisionSources);

        Optional<EstimatedRobotPoses> startingPose = m_vision.getPose();

        m_poseEstimator = new SwerveDrivePoseEstimator(
            m_kinematics,
            getHeading(),
            getPositions(),
            startingPose.isEmpty() ? new Pose2d() : startingPose.get().getPose2d()
        );
        
        // m_autoAimPID = new PIDController(0.25, 0, 0.27); // works pretty well
        m_yawPIDController = new PIDController(0.25, 0, 0.3); // works pretty well
        
        // m_autoAimPID = new PIDController(0.18, 0, 0.27);
        m_yawPIDController.setIZone(.5);
        m_yawPIDController.setIntegratorRange(-1, 1);
        m_yawPIDController.setI(0.2);
        
        m_yawPIDController.enableContinuousInput(-180, 180);

        // TODO: Tolerance
        m_yawPIDController.setTolerance(1);
        
        m_autoAiming = false;

        m_reachedAutoAimSetpointTrigger = new Trigger(() -> m_autoAiming && m_yawPIDController.atSetpoint());
        
        m_autoAimAngleLog = new LoggableDouble("AutoAimPIDAngle", true, false, null);
        m_autoAimPIDSetpointLog = new LoggableDouble("AutoAimPIDSetpoint", true, false, null);
        m_autoAimStateLog = new LoggableBoolean("AutoAimState", true, true, () -> m_autoAiming);
        m_poseLog = new LoggablePose2d("RobotPose", true, true, this::getPose);
        m_moduleStatesLog = new LoggableSwerveModuleStates("SwerveModuleStatesCurrent", true, true, this::getStates);
        m_moduleDesiredStatesLog = new LoggableSwerveModuleStates("SwerveModuleStatesDesired", true);

        Logger.addLoggable(m_poseLog);
        Logger.addLoggable(m_moduleStatesLog);
        Logger.addLoggable(m_moduleDesiredStatesLog);

        AutoBuilder.configureHolonomic(
            () -> getPose(),
            (Pose2d pose) -> resetPose(pose),
            () -> getSpeeds(),
            (ChassisSpeeds speeds) -> drive(speeds, false),
            Constants.SwerveConstants.AutonConstants.kHolonomicPathFollowerConfig,
            () -> false,
            this
        );

        this.coast();
    }

    /** drive with desired x/y/rot velocities */
    public void drive(double vx, double vy, double vrot, boolean fieldRelative) {
        m_state.addState(SwerveStateEnum.DRIVING);
        if (m_autoAiming && fieldRelative)
            vrot = calcAutoAim();
        else
            m_autoAimStateLog.log(false);
        


        Rotation2d navXVal = getHeading();
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, vrot, navXVal) : new ChassisSpeeds(vx, vy, vrot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.kMaxSpeed);

        m_moduleFL.setDesiredState(swerveModuleStates[0]);
        m_moduleFR.setDesiredState(swerveModuleStates[1]);
        m_moduleBL.setDesiredState(swerveModuleStates[2]);
        m_moduleBR.setDesiredState(swerveModuleStates[3]);

        m_moduleDesiredStatesLog.log(swerveModuleStates);
    }

    /** drive with desired chassis speeds */
    public void drive(ChassisSpeeds desiredSpeeds, boolean fieldRelative) {
        drive(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond, desiredSpeeds.omegaRadiansPerSecond, fieldRelative);
    }



    /** Brake on all motors on all swerve modules */
    private void brakeAll() {
        m_state.addState(SwerveStateEnum.BRAKE_ALL);
        m_moduleFL.brakeAll();
        m_moduleFL.brakeAll();
        m_moduleFL.brakeAll();
        m_moduleFL.brakeAll();
    }

    /** Coast on all motors on all swerve modules */
    public void coast() {
        m_state.addState(SwerveStateEnum.COAST_ALL);
        m_moduleFL.coastAll();
        m_moduleFR.coastAll();
        m_moduleBL.coastAll();
        m_moduleBR.coastAll();
    }

    public Command getOrientModulesCommand(ModuleOrientation orientation) {
        return this.run(() -> orientModules(orientation));
    }

    public void orientModules(ModuleOrientation orientation) {

        switch (orientation) {
            case CIRCLE:
                SwerveModuleState flBrCircle = new SwerveModuleState();
                flBrCircle.speedMetersPerSecond = 0.0;
                flBrCircle.angle = Rotation2d.fromRadians(-Math.PI / 4);
        
                SwerveModuleState frBlCircle = new SwerveModuleState();
                frBlCircle.speedMetersPerSecond = 0.0;
                frBlCircle.angle = Rotation2d.fromRadians(Math.PI / 4);
        
                m_moduleFL.setDesiredState(flBrCircle);
                m_moduleFR.setDesiredState(frBlCircle);
                m_moduleBR.setDesiredState(flBrCircle);
                m_moduleBL.setDesiredState(frBlCircle);
        
                m_moduleDesiredStatesLog.log(new SwerveModuleState[] {flBrCircle, frBlCircle, frBlCircle, flBrCircle});
        

                break;
            case BRAKE_X:
                SwerveModuleState flBrBrake = new SwerveModuleState();
                flBrBrake.speedMetersPerSecond = 0.0;
                flBrBrake.angle = Rotation2d.fromRadians(Math.PI / 4);
                
                SwerveModuleState frBlBrake = new SwerveModuleState();
                frBlBrake.speedMetersPerSecond = 0.0;
                frBlBrake.angle = Rotation2d.fromRadians(-Math.PI / 4);
                
                m_moduleFL.setDesiredState(flBrBrake);
                m_moduleFR.setDesiredState(frBlBrake);
                m_moduleBL.setDesiredState(frBlBrake);
                m_moduleBR.setDesiredState(flBrBrake);
        
                m_moduleDesiredStatesLog.log(new SwerveModuleState[] {flBrBrake, frBlBrake, frBlBrake, flBrBrake});
        
                brakeAll();
                break;
            case ALIGNED:
                SwerveModuleState alignedState = new SwerveModuleState();
                alignedState.speedMetersPerSecond = 0.0;
                alignedState.angle = Rotation2d.fromRadians(0);

                m_moduleFL.setDesiredState(alignedState);
                m_moduleFR.setDesiredState(alignedState);
                m_moduleBL.setDesiredState(alignedState);
                m_moduleBR.setDesiredState(alignedState);

                m_moduleDesiredStatesLog.log(new SwerveModuleState[] {alignedState,alignedState,alignedState,alignedState});
        }
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
            boolean isFieldRelative = fieldRelative.getAsBoolean();
            double vx = MathUtil.applyDeadband(joyLeftX.getAsDouble(), Constants.kJoystickDeadzone);
            double vy = MathUtil.applyDeadband(joyLeftY.getAsDouble(), Constants.kJoystickDeadzone);
            double vrot = MathUtil.applyDeadband(joyRightX.getAsDouble(), Constants.kJoystickDeadzone);

            
            vx *= SwerveConstants.kMaxSpeed;
            vy *= SwerveConstants.kMaxSpeed;
            vrot *= SwerveConstants.kMaxAngularSpeed;
            if (!isFieldRelative) {
                vx *= 0.15;
                vy *= 0.15;
                vrot *= 0.15;
                
                vx = vx + vy;
                vy = vx - vy;
                vx = vx - vy;
            }

            drive(vx, vy, vrot, isFieldRelative);
        }).withName("DriveWithJoystickCommand");
    }

    public Command getAlignWheelCommand() {
        return this.run(() -> {
            SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));
            SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveConstants.kMaxSpeed);
            
            m_moduleBL.getState().angle.getRotations();
            
            states[0].angle = Rotation2d.fromRadians(0);
            states[1].angle = Rotation2d.fromRadians(0);
            states[2].angle = Rotation2d.fromRadians(0);
            states[3].angle = Rotation2d.fromRadians(0);

            m_moduleFL.setDesiredState(states[0], false);
            m_moduleFR.setDesiredState(states[1], false);
            m_moduleBL.setDesiredState(states[2], false);
            m_moduleBR.setDesiredState(states[3], false);

            m_moduleDesiredStatesLog.log(states);
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


    public Rotation2d getHeading() {
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

    public double[] getWheelPoses() {
        return (new double[] {m_moduleFL.getWheelPos(),m_moduleFR.getWheelPos(),m_moduleBL.getWheelPos(),m_moduleBR.getWheelPos()});
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

        Optional<EstimatedRobotPoses> poses = m_vision.getPose();

        if (poses.isEmpty()) {
            return;
        }

        for (EstimatedRobotPose pose : poses.get().getEstimatedRobotPoses()) {
            m_poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
        }
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    private void resetPose(Pose2d pose) {
        m_poseEstimator.resetPosition(getHeading(), getPositions(), pose);
    }

    public void setAutoAim(boolean autoAim) { 
        m_autoAiming = autoAim;
    }

    public void toggleAutoAim() {
        m_autoAiming = !m_autoAiming;
    }

    /**
     * 
     * @return a command that toggles auto aiming to speaker
     */
    public Command getToggleAutoAimCommand() {
        return this.runOnce(() -> toggleAutoAim()).withName("toggleAutoAimCommand");
    }

    public Command getStartAutoAimCommand() {
        return this.runOnce(() -> m_autoAiming = true).withName("enableAutoAimCommand");
    }

    public Command getStopAutoAimCommand() {
        return this.runOnce(() -> m_autoAiming = false).withName("disableAutoAimCommand");
    }

    /**
     * Calculates a desired rotation velocity that will automatically align the bot with the respective alliance's speaker
     */
    private double calcAutoAim() {
        Pose2d robotPose = getPose();

        double angle = robotPose.getRotation().getDegrees() % 360;
        m_autoAimAngleLog.log(angle);
        m_autoAimStateLog.log(true);

        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isEmpty() || alliance.get() == Alliance.Red) {
            double angleOffset = 0;

            Pose2d redPose = new Pose2d(
                Units.inchesToMeters(652.73),
                Units.inchesToMeters(218.42),
                Rotation2d.fromRadians(0)
            );

            Transform2d delta = new Transform2d(
                redPose.getX() - robotPose.getX(),
                redPose.getY() - robotPose.getY(),
                Rotation2d.fromRadians(0)
            );
            
            double setpoint = Units.radiansToDegrees(Math.atan(delta.getY() / delta.getX()));

            m_yawPIDController.setSetpoint(setpoint);
            m_autoAimPIDSetpointLog.log(setpoint);
            
            double pidOutput = m_yawPIDController.calculate(angle - angleOffset);

            return pidOutput;
        }
        else {
            // double angleOffset = 180;

            // // blue speaker tag pose
            // Pose2d bluePose = new Pose2d(
            //     Units.inchesToMeters(-1.5),
            //     Units.inchesToMeters(218.42),
            //     Rotation2d.fromRadians(0)
            // );

            // Transform2d delta = new Transform2d(
            //     robotPose.getX() - bluePose.getX(), 
            //     robotPose.getY() - bluePose.getY(), 
            //     Rotation2d.fromRadians(0)
            // );
          
            // double setpoint = Units.radiansToDegrees(Math.atan(delta.getY() / delta.getX()));

            // m_autoAimPID.setSetpoint(setpoint);
            // m_autoAimPIDSetpointLog.log(setpoint);

            // double pidOutput = m_autoAimPID.calculate(angle - angleOffset);
            
            // return pidOutput;
            double angleOffset = 180;

            // blue speaker tag pose
            Pose2d bluePose = new Pose2d(
                Units.inchesToMeters(-1.5),
                Units.inchesToMeters(218.42),
                Rotation2d.fromRadians(0)
            );

            Transform2d delta = new Transform2d(
                robotPose.getX() - bluePose.getX(), 
                robotPose.getY() - bluePose.getY(), 
                Rotation2d.fromRadians(0)
            );
            
            double setpoint = Units.radiansToDegrees(Math.atan(delta.getY() / delta.getX()));

            m_yawPIDController.setSetpoint(setpoint);
            m_autoAimPIDSetpointLog.log(setpoint);
            
            double pidOutput = m_yawPIDController.calculate(angle + angleOffset);

            return pidOutput;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("vx", getSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("vy", getSpeeds().vyMetersPerSecond);
        SmartDashboard.putNumber("vrot", getSpeeds().omegaRadiansPerSecond);

        updatePose();

        Pose2d pose = getPose();
        SmartDashboard.putNumber("posx", pose.getX());
        SmartDashboard.putNumber("posy", pose.getY());

        SmartDashboard.putNumber("AutoAimVRot", calcAutoAim());

        SmartDashboard.putNumber("ModFLAngle", m_moduleFL.getState().angle.getDegrees());
        SmartDashboard.putNumber("ModFRAngle", m_moduleFR.getState().angle.getDegrees());
        SmartDashboard.putNumber("ModBLAngle", m_moduleBL.getState().angle.getDegrees());
        SmartDashboard.putNumber("ModBRAngle", m_moduleBR.getState().angle.getDegrees());

        SmartDashboard.putBoolean("AutoAiming", m_autoAiming);
    }
}
