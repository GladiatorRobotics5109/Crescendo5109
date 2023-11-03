package frc.robot;

public class SwerveConstants {
    private final SwerveModule m_frontLeft;
    private final SwerveModule m_frontRight;
    private final SwerveModule m_backLeft;
    private final SwerveModule m_backRight;

    private final double m_maxAngularSpeed;
    private final double m_maxSpeed;

    private final AHRS m_navX;

    public SwerveConstants(
        double maxAngularSpeed, 
        double maxSpeed,
        SwerveModule frontLeft, 
        SwerveModule frontRight, 
        SwerveModule backLeft, 
        SwerveModule backRight,
        AHRS navX) {
            m_maxAngularSpeed = maxAngularSpeed;
            m_maxSpeed = maxSpeed;

            m_frontLeft = frontLeft;
            m_frontRight = frontRight;
            m_backLeft = backLeft;
            m_backRight = backRight;

            m_navX = navX;
    }


    public SwerveModule getFrontLeft() {
        return m_frontLeft;
    }
    public SwerveModule getFrontRight() {
        return m_frontRight;
    }
    public SwerveModule getBackLeft() {
        return m_backLeft;
    }
    public SwerveModule getBackRight() {
        return m_backRight;
    }

    public AHRS getNavX() {
        return m_navX;
    }
}
