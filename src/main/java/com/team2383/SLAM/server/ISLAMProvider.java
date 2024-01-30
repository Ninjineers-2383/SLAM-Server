package com.team2383.SLAM.server;

import com.team2383.SLAM.server.common.buffer.BufferEntry;

import edu.wpi.first.math.geometry.Pose3d;

public interface ISLAMProvider {
    public void addEntry(BufferEntry entry);

    public Pose3d getLatestPose();

    public double getLatestPoseTime();
}
