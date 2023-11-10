package frc.robot.subsystems.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {
    // swerve constants
    private final SwerveConstants m_swerveConstants;

    // swerve modules
    private final SwerveModule m_moduleFL;
    private final SwerveModule m_moduleFR;
    private final SwerveModule m_moduleBL;
    private final SwerveModule m_moduleBR;

    private double m_maxSpeed;
    private double m_maxAngularSpeed;

    private final SwerveDriveKinematics m_kinematics;

    private AHRS m_navX;

    public SwerveSubsystem(SwerveConstants swerveConstants) {
        m_swerveConstants = swerveConstants;

        m_moduleFL = m_swerveConstants.getFrontLeft();
        m_moduleFR = m_swerveConstants.getFrontRight();
        m_moduleBL = m_swerveConstants.getBackLeft();
        m_moduleBR = m_swerveConstants.getBackRight();
        
        m_kinematics = new SwerveDriveKinematics(
            m_moduleFL.getPos(), 
            m_moduleFR.getPos(),
            m_moduleBL.getPos(), 
            m_moduleBR.getPos());

        m_maxSpeed = swerveConstants.getMaxSpeed();
        m_maxAngularSpeed = swerveConstants.getMaxAngluarSpeed();
    
        m_navX = swerveConstants.getNavX();
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
        Rotation2d navXVal = new Rotation2d((m_navX.getAngle()% 360) * Math.PI / 180);
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond, desiredSpeeds.omegaRadiansPerSecond, navXVal) : desiredSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, m_maxSpeed);
        
        m_moduleFL.setDesiredState(swerveModuleStates[0]);
        m_moduleFR.setDesiredState(swerveModuleStates[1]);
        m_moduleBL.setDesiredState(swerveModuleStates[2]);
        m_moduleBR.setDesiredState(swerveModuleStates[3]);
    }

    public void setMaxSpeed(double speed) {
        m_maxSpeed = speed;
    }

    /** Brake and X the wheels to stay still */
    public void brakeAndX() {
        SwerveModuleState flbr = new SwerveModuleState();
        flbr.speedMetersPerSecond = 0.0;
        flbr.angle = Rotation2d.fromRadians(-Math.PI / 4);

        m_moduleFL.setDesiredState(flbr);
        m_moduleBR.setDesiredState(flbr);

        SwerveModuleState frbl = new SwerveModuleState();
        frbl.speedMetersPerSecond = 0.0;
        frbl.angle = Rotation2d.fromRadians(Math.PI / 4);

        m_moduleFR.setDesiredState(frbl);
        m_moduleBL.setDesiredState(frbl);

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
        BooleanSupplier fieldRelative) {
        return run(() -> {
            // get joystick axises
            double vx = MathUtil.applyDeadband(joyLeftX.getAsDouble(), Constants.kJoystickDeadzone);
            double vy = MathUtil.applyDeadband(joyLeftY.getAsDouble(), Constants.kJoystickDeadzone);
            double rot = MathUtil.applyDeadband(joyRightX.getAsDouble(), Constants.kJoystickDeadzone);

            // apply max speeds
            vx *= m_maxSpeed;
            vy *= m_maxSpeed;
            rot *= m_maxAngularSpeed;

            drive(vx, vy, rot, fieldRelative.getAsBoolean());
        }).withName("DriveWithJoystickCommand");
    }
}
