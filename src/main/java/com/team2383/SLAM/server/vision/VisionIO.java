package com.team2383.SLAM.server.vision;

import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {
    public static class VisionIOInputs {
        public double[] timestamps = new double[] {};
        public double[][] frames = new double[][] {};
        public long fps = 0;
    }

    public default void updateInputs(VisionIOInputs inputs, Pose3d robotPose) {
    }
}
