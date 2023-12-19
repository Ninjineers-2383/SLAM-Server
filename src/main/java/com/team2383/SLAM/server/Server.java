package com.team2383.SLAM.server;

import java.util.List;
import java.util.Timer;

import com.team2383.SLAM.server.SLAM.EKFSLAM;
import com.team2383.SLAM.server.helpers.ChassisSpeedsStruct;
import com.team2383.SLAM.server.vision.VisionIONorthstar;
import com.team2383.SLAM.server.vision.VisionSubsystem;
import com.team2383.SLAM.server.vision.VisionSubsystem.TimestampVisionUpdate;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArraySubscriber;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;

public class Server {
    private EKFSLAM m_slam;
    private VisionSubsystem m_visionSubsystem;

    private int previousNumLandmarks = 0;

    private IntegerSubscriber numLandmarksSub;
    private StructArraySubscriber<Pose3d> landmarksSub;
    private StructSubscriber<ChassisSpeeds> chassisSpeedsSub;

    private StructPublisher<Pose3d> posePub;
    private StructArrayPublisher<Pose3d> landmarksPub;

    private int landmarkNums;
    private Pose3d[] landmarks;
    private ChassisSpeeds chassisSpeeds;

    public Server() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("slam_data");
        landmarksSub = table.getStructArrayTopic("landmarks", Pose3d.struct).subscribe(new Pose3d[0]);
        numLandmarksSub = table.getIntegerTopic("numLandmarks").subscribe(0);
        chassisSpeedsSub = table.getStructTopic("chassisSpeeds", new ChassisSpeedsStruct()).subscribe(new ChassisSpeeds());

        posePub = table.getStructTopic("pose", Pose3d.struct).publish();
        landmarksPub = table.getStructArrayTopic("landmarks", Pose3d.struct).publish();
    }

    public void loop() {
        landmarkNums = (int) numLandmarksSub.get(0);
        landmarks = landmarksSub.get(new Pose3d[0]);
        chassisSpeeds = chassisSpeedsSub.get(new ChassisSpeeds());

        if (landmarkNums != previousNumLandmarks || m_slam == null) {
            m_slam = new EKFSLAM(landmarkNums);
            m_slam.seedLandmarks(landmarks);

            m_visionSubsystem = new VisionSubsystem(
                landmarks, 
                new VisionIONorthstar("northstar1"), new VisionIONorthstar("northstar2"));

            m_visionSubsystem.setPoseSupplier(m_slam::getRobotPose);
            m_visionSubsystem.setVisionConsumer(this::visionConsumer);
        }

        previousNumLandmarks = landmarkNums;

        m_slam.predict(chassisSpeeds, 0.02);

        posePub.set(m_slam.getRobotPose());
        landmarksPub.set(m_slam.getLandmarkPoses());
    }

    public void visionConsumer(List<TimestampVisionUpdate> visionUpdates) {
        Pose3d pose = m_slam.getRobotPose();

        Pose3d[] landmarks = new Pose3d[visionUpdates.size()];
        int i = 0;
        for (TimestampVisionUpdate update : visionUpdates) {
            if (!m_slam.isEnabled()) {
                m_slam.setInitialRobotPose(landmarks[update.tagId()].plus(update.pose().inverse()));
            }
            Pose3d tagPose = pose.plus(update.pose());
            landmarks[i] = tagPose;
            m_slam.correct(update.pose(), update.tagId() - 1);
            i++;
        }
    }
}
