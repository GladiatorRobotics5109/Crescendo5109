package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveConstants;
import frc.robot.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {
    // swerve constants
    private final SwerveConstants m_swerveConstants;

    // swerve modules
    private final SwerveModule m_moduleFL;
    private final SwerveModule m_moduleFR;
    private final SwerveModule m_moduleBL;
    private final SwerveModule m_moduleBR;

    private final double m_maxSpeed;

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
        
            m_navX = swerveConstants.getNavX();
    }

    public void drive(double vx, double vy, double rot, boolean fieldRelative) {
        Rotation2d navXVal = new Rotation2d((-m_navX.getAngle() % 360) * Math.PI / 180);
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rot, navXVal) : new ChassisSpeeds(vx, vy, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, m_maxSpeed);

        m_moduleFL.setDesiredState(swerveModuleStates[0]);
        m_moduleFR.setDesiredState(swerveModuleStates[1]);
        m_moduleBL.setDesiredState(swerveModuleStates[2]);
        m_moduleBR.setDesiredState(swerveModuleStates[3]);
    }
}
