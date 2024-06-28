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
        return Pose2d.struct.getSize() + Double.BYTES + Integer.BYTES;
    }

    @Override
    public String getSchema() {
        // Strings aren't supported by struct serialization (I think), so cameraName
        // cannot be serialized
        return "Pose2d estimatedPose; double timestamp; bool isFromSimCamera";
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
        // String cameraName = getString(bb, Pose2d.struct.getSize() + Double.BYTES);
        boolean isFromSimCamera = bb.getInt(Pose2d.struct.getSize() + Double.BYTES) == 1;

        return new VisionMeasurement(estimatedPose, timestamp, "unknown", isFromSimCamera);
    }

    @Override
    public void pack(ByteBuffer bb, VisionMeasurement visionMeasurement) {
        Pose2d.struct.pack(bb, visionMeasurement.getEstimatedPose());
        bb.putDouble(visionMeasurement.getTimestamp());
        // putString(bb, visionMeasurement.getCameraName());
        bb.putInt(visionMeasurement.isFromSimCamera() ? 1 : 0);
    }

    // /**
    // * Encodes a string in the format (int)length + (byte[])data
    // */
    // private void putString(ByteBuffer bb, String str) {
    // if (str.getBytes().length >= kMaxStringSize) {
    // DriverStation.reportWarning(
    // "The String '" + str
    // + "' exceeds the max string size for a VisionMeasurementStruct, it cannot be
    // serialized. Consider increasing VisionMeasurementStruct.kMaxStringSize or
    // shortening the length of the string",
    // null
    // );
    // str = "String cannot be serialized.";
    // }
    // bb.putInt(str.length());
    // bb.put(str.getBytes());
    // }

    // /**
    // * Decodes a string in the format (int)length + (byte[])data
    // */
    // private String getString(ByteBuffer bb, int index) {
    // int size = bb.getInt(index);
    // StringBuilder sb = new StringBuilder();
    //
    // for (int i = 1; i <= size; i++) {
    // sb.append(bb.getChar(index + i));
    // }
    //
    // return sb.toString();
    // }
}
