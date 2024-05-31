package frc.robot.subsystems.vision;

import java.nio.ByteBuffer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.struct.Struct;

public class VisionMeasurementStruct implements Struct<VisionMeasurement> {
    @Override
    public Class<VisionMeasurement> getTypeClass() {
        return VisionMeasurement.class;
    }

    @Override
    public String getTypeString() {
        return "struct:VisionMeasurement";
    }

    @Override
    public int getSize() {
        return Pose2d.struct.getSize() + Double.BYTES;
    }

    @Override
    public String getSchema() {
        return "Pose2d estimatedPose; double timestamp";
    }

    @Override
    public Struct<?>[] getNested() {
        return new Struct[] {
            Pose2d.struct
        };
    }

    @Override
    public VisionMeasurement unpack(ByteBuffer bb) {
        Pose2d estimatedPose = Pose2d.struct.unpack(bb);
        double timestamp = bb.getDouble(Pose2d.struct.getSize());

        return new VisionMeasurement(estimatedPose, timestamp);
    }

    @Override
    public void pack(ByteBuffer bb, VisionMeasurement value) {
        Pose2d.struct.pack(bb, value.getEstimatedPose());
        bb.putDouble(value.getTimestamp());
    }

    // From:
    // https://stackoverflow.com/questions/13071777/convert-double-to-byte-array
    // private static byte[] doubleToBytes(double d) {
    // byte[] bytes = new byte[8];
    // long bits = Double.doubleToLongBits(d);
    // for (int i = 0; i < 8; i++) {
    // bytes[i] = (byte) ((bits >> ((7 - i) * 8)) & 0xff);
    // }

    // return bytes;
    // }
}
