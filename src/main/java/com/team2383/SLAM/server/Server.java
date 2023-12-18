package com.team2383.SLAM.server;

import com.team2383.SLAM.server.SLAM.EKFSLAM;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;import edu.wpi.first.networktables.StructArraySubscriber;

public class Server {
    private EKFSLAM m_slam;

    private int previousNumLandmarks = 0;

    private IntegerSubscriber numLandmarksSub;
    private StructArraySubscriber<Pose3d> landmarksSub;

    private int landmarkNums;
    private Pose3d[] landmarks;

    public Server() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("slam_data");
        landmarksSub = table.getStructArrayTopic("landmarks", Pose3d.struct).subscribe(new Pose3d[0]);
        numLandmarksSub = table.getIntegerTopic("numLandmarks").subscribe(0);
    }

    public void loop() {
        landmarkNums = (int) numLandmarksSub.get();
        landmarks = landmarksSub.get();

        if (landmarkNums != previousNumLandmarks) {
            m_slam = new EKFSLAM(landmarkNums);
            m_slam.seedLandmarks(landmarks);
        }
        previousNumLandmarks = landmarkNums;

    }
}
