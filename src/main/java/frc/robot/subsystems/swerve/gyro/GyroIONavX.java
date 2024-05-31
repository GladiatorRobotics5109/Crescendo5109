package frc.robot.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.AHRS;

public class GyroIONavX implements GyroIO {
    private final AHRS m_navX;

    public GyroIONavX() {
        m_navX = new AHRS(SPI.Port.kMXP);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.isConnected = m_navX.isConnected();
        inputs.pitch = Rotation2d.fromDegrees(m_navX.getPitch());
        inputs.roll = Rotation2d.fromDegrees(m_navX.getRoll());
        inputs.yaw = m_navX.getRotation2d();
    }
}
