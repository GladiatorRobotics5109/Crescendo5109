package frc.robot.hardware.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOSim implements GyroIO {
    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.isConnected = true;
        inputs.isSim = true;
        inputs.pitch = inputs.pitch == null ? Rotation2d.fromRotations(0) : inputs.pitch;
        inputs.roll = inputs.roll == null ? Rotation2d.fromRotations(0) : inputs.roll;
        inputs.yaw = inputs.yaw == null ? Rotation2d.fromRotations(0) : inputs.yaw;
    }
}
