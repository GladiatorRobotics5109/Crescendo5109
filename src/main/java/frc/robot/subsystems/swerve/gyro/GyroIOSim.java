package frc.robot.subsystems.swerve.gyro;

public class GyroIOSim implements GyroIO {
    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.isConnected = true;
        inputs.isSim = true;
        inputs.pitch = inputs.pitch;
        inputs.roll = inputs.roll;
        inputs.yaw = inputs.yaw;
    }
}
