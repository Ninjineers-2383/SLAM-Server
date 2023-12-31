package com.team2383.SLAM.server.vision;

import com.team2383.SLAM.server.vision.VisionIO.VisionIOInputs;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.List;
import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class VisionSubsystem {
    private final VisionIO[] visionIO;
    private final VisionIOInputs[] inputs;

    private Consumer<List<TimestampVisionUpdate>> visionConsumer = (x) -> {
    };
    private Supplier<Pose3d> poseSupplier = () -> new Pose3d();

    private Pose3d[] tagPoses;

    private Transform3d[] camTransforms;
    // private double varianceScale;
    // private double varianceStatic;

    public VisionSubsystem(Pose3d[] tagPoses, VisionIO... visionIO) {
        this.tagPoses = tagPoses;
        this.visionIO = visionIO;
        this.inputs = new VisionIOInputs[visionIO.length];
        for (int i = 0; i < visionIO.length; i++) {
            inputs[i] = new VisionIOInputs();
        }

    }

    public void periodic() {
        Pose3d robotPose = poseSupplier.get();
        for (int i = 0; i < visionIO.length; i++) {
            visionIO[i].updateInputs(inputs[i], robotPose);
        }

        List<TimestampVisionUpdate> updates = new ArrayList<>();
        for (int i = 0; i < inputs.length; i++) {
            for (int j = 0; j < inputs[i].frames.length; j++) {
                double timestamp = inputs[i].timestamps[j];

                double[] values = inputs[i].frames[j];

                switch ((int) values[0]) {
                    case 1: // 1 pose estimate
                        Pose3d pose = new Pose3d(
                                values[2], values[3], values[4],
                                new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
                        int tag1 = (int) values[9], tag2 = (int) values[10];
                        Transform3d offset1 = new Transform3d(pose, tagPoses[tag1]);
                        Transform3d offset2 = new Transform3d(pose, tagPoses[tag2]);

                        updates.add(new TimestampVisionUpdate(timestamp, offset1, tag1));
                        updates.add(new TimestampVisionUpdate(timestamp, offset2, tag2));

                    case 2: // 2 transform estimates
                        double error1 = values[1];
                        Transform3d transform1 = new Transform3d(
                                values[2], values[3], values[4],
                                new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
                        double error2 = values[9];
                        Transform3d transform2 = new Transform3d(
                                values[10], values[11], values[12],
                                new Rotation3d(new Quaternion(values[13], values[14], values[15], values[16])));
                        int tag = (int) values[17];

                        if (error1 < error2) {
                            updates.add(new TimestampVisionUpdate(timestamp,
                                    camTransforms[i].plus(transform1), tag));
                        } else {
                            updates.add(new TimestampVisionUpdate(timestamp,
                                    camTransforms[i].plus(transform2), tag));
                        }
                        break;
                }
            }
        }

        visionConsumer.accept(updates);
    }

    public void setVisionConstants(Transform3d[] camTransforms, double varianceScale, double varianceStatic) {
        this.camTransforms = camTransforms;
        // this.varianceScale = varianceScale;
        // this.varianceStatic = varianceStatic;
    }

    public void setVisionConsumer(Consumer<List<TimestampVisionUpdate>> consumer) {
        this.visionConsumer = consumer;
    }

    public void setPoseSupplier(Supplier<Pose3d> supplier) {
        this.poseSupplier = supplier;
    }

    /** Represents a single vision pose */
    public static record TimestampVisionUpdate(double timestamp, Transform3d pose, int tagId) {
    }

}
