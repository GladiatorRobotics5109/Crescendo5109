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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.util.Constants;
import frc.robot.Robot;
import frc.robot.util.Constants.SwerveConstants;
import frc.robot.util.logging.LoggableBoolean;
import frc.robot.util.logging.LoggableDouble;
import frc.robot.util.logging.Logger;
import frc.robot.vision.EstimatedRobotPoses;
import frc.robot.vision.VisionManager;

public class SwerveSubsystem extends SubsystemBase {
    // swerve modules
    private final SwerveModuleKrakenTurnKrakenDrive m_moduleFL;
    private final SwerveModuleKrakenTurnKrakenDrive m_moduleFR;
    private final SwerveModuleKrakenTurnKrakenDrive m_moduleBL;
    private final SwerveModuleKrakenTurnKrakenDrive m_moduleBR;
    
    private final double m_defaultSpeed;
    private double m_currentSpeed;
    private double m_maxAngularSpeed;

    private final SwerveDriveKinematics m_kinematics;

    private final VisionManager m_vision;

    private final SwerveDrivePoseEstimator m_poseEstimator;
    
    private final AHRS m_navX;
    
    private final PIDController m_autoAimPID;
    private boolean m_autoAiming;

    private final LoggableDouble m_autoAimAngleLog;
    private final LoggableDouble m_autoAimPIDSetpointLog;
    private final LoggableBoolean m_autoAimStateLog;
    private final LoggablePose2d m_poseLog;
    private final LoggableDouble m_currentMaxSpeedLog;

    private final LoggableSwerveModuleStates m_moduleStatesLog;
    private final LoggableSwerveModuleStates m_moduleDesiredStatesLog;
    private final LoggableSwerveModuleStates m_moduleDesiredStatesNotOptimizedLog;
    
    private final SwerveState m_state;

    private final Trigger m_reachedAutoAimSetpointTrigger;

    public SwerveSubsystem() {
        m_state = StateMachine.getSwerveState();

        // TODO: select right CAN ids for motors
        m_moduleFL = new SwerveModuleKrakenTurnKrakenDrive(Constants.SwerveConstants.kModulePosFrontLeft, "frontLeft", 0, 2, 5);
        m_moduleFR = new SwerveModuleKrakenTurnKrakenDrive(Constants.SwerveConstants.kModulePosFrontRight, "frontRight", 1, 4, 40);
        m_moduleBL = new SwerveModuleKrakenTurnKrakenDrive(Constants.SwerveConstants.kModulePosBackLeft, "backLeft", 2, 1, 22);
        m_moduleBR = new SwerveModuleKrakenTurnKrakenDrive(Constants.SwerveConstants.kModulePosBackRight, "backRight", 3, 3, 30);
    
        m_navX = SwerveConstants.kNavX;
        m_navX.reset();
        m_navX.setAngleAdjustment(180);
        
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

        m_defaultSpeed = SwerveConstants.kMaxSpeed;
        m_currentSpeed = m_defaultSpeed;
        m_maxAngularSpeed = SwerveConstants.kMaxAngularSpeed;
        
        // m_autoAimPID = new PIDController(0.25, 0, 0.27); // works pretty well
        m_autoAimPID = new PIDController(0.25, 0, 0.3); // works pretty well
        // m_autoAimPID = new PIDController(0.18, 0, 0.27);
        m_autoAimPID.setIZone(.5);
        m_autoAimPID.setIntegratorRange(-1, 1);
        m_autoAimPID.setI(0.2);
        m_autoAimPID.enableContinuousInput(-180, 180);

        // TODO: Tolerance
        m_autoAimPID.setTolerance(1);
        
        m_autoAiming = false;

        m_reachedAutoAimSetpointTrigger = new Trigger(() -> m_autoAiming && m_autoAimPID.atSetpoint());
        
        m_autoAimAngleLog = new LoggableDouble("AutoAimPIDAngle", true, false, null);
        m_autoAimPIDSetpointLog = new LoggableDouble("AutoAimPIDSetpoint", true, false, null);
        m_autoAimStateLog = new LoggableBoolean("AutoAimState", true, true, () -> m_autoAiming);
        m_poseLog = new LoggablePose2d("RobotPose", true, true, this::getPose);
        m_moduleStatesLog = new LoggableSwerveModuleStates("SwerveModuleStatesCurrent", true);
        m_moduleDesiredStatesLog = new LoggableSwerveModuleStates("SwerveModuleStatesDesired", true);
        m_moduleDesiredStatesNotOptimizedLog = new LoggableSwerveModuleStates("SwerveModuleStatesDesiredNotOptimized", true);
        m_currentMaxSpeedLog = new LoggableDouble("Current Max Speed", true, true, () -> m_currentSpeed);

        Logger.addLoggable(m_poseLog);
        Logger.addLoggable(m_moduleStatesLog);
        Logger.addLoggable(m_moduleDesiredStatesLog);
        Logger.addLoggable(m_currentMaxSpeedLog);

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
        if (m_autoAiming)
            vrot = calcAutoAim();
        else
            m_autoAimStateLog.log(false);

        // vx *= Math.cos(vrot / (SwerveConstants.kMaxAngularSpeed / 2));
        // vy *= Math.cos(vrot / (SwerveConstants.kMaxAngularSpeed / 2));
      
        // Rotation2d navXVal = Rotation2d.fromDegrees(getHeading().getDegrees() + 90);
        Optional<Alliance> alliance = DriverStation.getAlliance();
        double offset = (alliance.isEmpty() || alliance.get() == Alliance.Red) ? -180 : 0; 
        Rotation2d rot = Rotation2d.fromDegrees(getPose().getRotation().getDegrees() + offset);
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(vy, vx, vrot, rot) : new ChassisSpeeds(vx, vy, vrot));
        m_moduleDesiredStatesNotOptimizedLog.log(swerveModuleStates);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, m_currentSpeed);

        m_moduleFL.setDesiredState(swerveModuleStates[0]);
        m_moduleFR.setDesiredState(swerveModuleStates[1]);
        m_moduleBL.setDesiredState(swerveModuleStates[2]);
        m_moduleBR.setDesiredState(swerveModuleStates[3]);

        // System.out.println("Driving: (" + vx + ", " + vy + ", " + vrot + ")");

        m_moduleDesiredStatesLog.log(swerveModuleStates);
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

        m_moduleDesiredStatesLog.log(new SwerveModuleState[] {flBr, frBl, frBl, flBr});

        brakeAll();
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
            double vrot = MathUtil.applyDeadband(joyRightX.getAsDouble(), Constants.kJoystickDeadzone);

            double right = joyRightTrigger.getAsDouble();
            double left = joyLeftTrigger.getAsDouble();

            double newSpeed = m_defaultSpeed + ((SwerveConstants.kMaxSpeed * 0.5) * (joyLeftTrigger.getAsDouble() - joyRightTrigger.getAsDouble()));
            setMaxSpeed(newSpeed);

            // apply max speeds
            vx *= newSpeed;
            vy *= newSpeed;
            vrot *= m_maxAngularSpeed;

            drive(vx, vy, vrot, fieldRelative.getAsBoolean());
        }).withName("DriveWithJoystickCommand");
    }

    public Command getAlignWheelCommand() {
        return this.runOnce(() -> {
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

    public SwerveModuleState[] getStates() {
        return new SwerveModuleState[] {
            m_moduleFL.getState(),
            m_moduleFR.getState(),
            m_moduleBL.getState(),
            m_moduleBR.getState()
        };
    }

    /*
     * For debug purposes
     */
    public SwerveModule[] getSwerveModules() {
        return new SwerveModule[] {
            m_moduleFL,
            m_moduleFR,
            m_moduleBL,
            m_moduleBR
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
            m_poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds, Constants.VisionConstants.kStdDevs);
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
        return this.runOnce(() -> {m_autoAiming = false; System.out.println("Stop Auto Aim");}).withName("disableAutoAimCommand");
    }

    public Command getResetPoseAllianceCommand() {
        return this.runOnce(() -> {
            Optional<Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isEmpty()) return;
            
            double offset = (alliance.get() == Alliance.Red) ? 0 : 180; 

            resetPose(
                new Pose2d(
                    0,
                    0,
                    Rotation2d.fromDegrees(offset)
                )
            );
            System.out.println("offset: " + offset);
        });
    }

    public Command getModuleTurnSysIdCommand() {
        SysIdRoutine fl = m_moduleFL.getTurnSysIdRoutine(this);
        SysIdRoutine fr = m_moduleFR.getTurnSysIdRoutine(this);
        SysIdRoutine bl = m_moduleBL.getTurnSysIdRoutine(this);
        SysIdRoutine br = m_moduleBR.getTurnSysIdRoutine(this);
        return Commands.sequence(
            fl.dynamic(Direction.kForward),
            fl.dynamic(Direction.kReverse),
            fl.quasistatic(Direction.kForward),
            fl.quasistatic(Direction.kReverse),
            // fr.dynamic(Direction.kForward),
            // fr.dynamic(Direction.kReverse),
            // fr.quasistatic(Direction.kForward),
            // fr.quasistatic(Direction.kReverse),
            // bl.dynamic(Direction.kForward),
            // bl.dynamic(Direction.kReverse),
            // bl.quasistatic(Direction.kForward),
            // bl.quasistatic(Direction.kReverse),
            // br.dynamic(Direction.kForward),
            // br.dynamic(Direction.kReverse),
            // br.quasistatic(Direction.kForward),
            // br.quasistatic(Direction.kReverse),
            Commands.print("Swerve Module SysId Complete!")
        );
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

            m_autoAimPID.setSetpoint(setpoint);
            m_autoAimPIDSetpointLog.log(setpoint);
            
            double pidOutput = m_autoAimPID.calculate(angle - angleOffset);

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

            m_autoAimPID.setSetpoint(setpoint);
            m_autoAimPIDSetpointLog.log(setpoint);
            
            double pidOutput = m_autoAimPID.calculate(angle + angleOffset);

            return pidOutput;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("vx", getSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("vy", getSpeeds().vyMetersPerSecond);
        SmartDashboard.putNumber("vrot", getSpeeds().omegaRadiansPerSecond);

        m_moduleStatesLog.log(getStates());
        // System.out.println("Cur angle: " + getStates()[0].angle.getDegrees());

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
