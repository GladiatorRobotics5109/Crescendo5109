package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOSim implements GyroIO {
    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.isConnected = true;
        inputs.pitch = Rotation2d.fromRotations(0);
        inputs.roll = Rotation2d.fromRotations(0);
        inputs.yaw = Rotation2d.fromRotations(0);
    }
}
