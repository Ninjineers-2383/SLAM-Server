package com.team2383.SLAM.server;

import java.util.List;

import com.team2383.SLAM.server.SLAM.TimeSyncedEKFSLAM;
import com.team2383.SLAM.server.vision.VisionIONorthstar;
import com.team2383.SLAM.server.vision.VisionSubsystem;
import com.team2383.SLAM.server.vision.VisionSubsystem.TimestampVisionUpdate;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSub;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArraySubscriber;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.networktables.TimestampedInteger;
import edu.wpi.first.networktables.TimestampedObject;

public class Server {
    private TimeSyncedEKFSLAM m_slam;
    private VisionSubsystem m_visionSubsystem;

    private int numLandmarks = 0;

    private IntegerSubscriber numLandmarksSub;
    private StructArraySubscriber<Pose3d> landmarksSub;
    private StructSubscriber<ChassisSpeeds> chassisSpeedsSub;

    private StructPublisher<Pose3d> posePub;
    private StructArrayPublisher<Pose3d> landmarksPub;
    private StructArrayPublisher<Pose3d> seenLandmarksPub;

    public Server() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("slam_data");

        landmarksSub = table.getStructArrayTopic("seed-landmarks", Pose3d.struct).subscribe(new Pose3d[0]);
        numLandmarksSub = table.getIntegerTopic("numLandmarks").subscribe(0);
        chassisSpeedsSub = table.getStructTopic("chassisSpeeds", ChassisSpeeds.struct)
                .subscribe(new ChassisSpeeds(), PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));

        posePub = table.getStructTopic("pose", Pose3d.struct).publish();
        landmarksPub = table.getStructArrayTopic("landmarks", Pose3d.struct).publish();
        seenLandmarksPub = table.getStructArrayTopic("seenLandmarks", Pose3d.struct).publish();

        reinitializeSLAM(numLandmarks, new Pose3d[0]);
    }

    public void loop() {
        TimestampedInteger[] newNumLandmarks = numLandmarksSub.readQueue();

        if (newNumLandmarks.length != 0) {
            numLandmarks = (int) newNumLandmarks[newNumLandmarks.length - 1].value;
            Pose3d[] landmarks = landmarksSub.get(new Pose3d[0]);
            reinitializeSLAM(numLandmarks, landmarks);

            System.out.println("Reinitialized SLAM with " + numLandmarks + " landmarks");
        }

        TimestampedObject<ChassisSpeeds>[] chassisSpeeds = chassisSpeedsSub.readQueue();

        for (TimestampedObject<ChassisSpeeds> chassisSpeed : chassisSpeeds) {
            m_slam.addDriveOdometryMeasurement(chassisSpeed.value, chassisSpeed.timestamp / 1000000.0);
        }

        m_visionSubsystem.periodic();

        posePub.set(m_slam.getRobotPose());
        landmarksPub.set(m_slam.getLandmarkPoses());
    }

    private void reinitializeSLAM(int numLandmarks, Pose3d[] landmarks) {
        m_slam = new TimeSyncedEKFSLAM(numLandmarks, landmarks);

        m_visionSubsystem = new VisionSubsystem(
                landmarks,
                new VisionIONorthstar("northstar-1")
        // new VisionIONorthstar("northstar-2")
        );

        m_visionSubsystem.setPoseSupplier(m_slam::getRobotPose);
        m_visionSubsystem.setVisionConsumer(this::visionConsumer);
    }

    public void visionConsumer(List<TimestampVisionUpdate> visionUpdates) {

        Pose3d[] seenLandmarks = new Pose3d[visionUpdates.size()];

        int i = 0;
        for (TimestampVisionUpdate update : visionUpdates) {
            m_slam.addVisionMeasurement(update.pose(), update.tagId() - 1, update.timestamp());

            Pose3d pose = m_slam.getRobotPose();
            if (pose == null) {
                continue;
            }
            Pose3d tagPose = pose.plus(update.pose());
            seenLandmarks[i] = tagPose;

            i++;
        }

        System.out.println("Vision updates: " + visionUpdates.size());

        seenLandmarksPub.set(seenLandmarks);
    }

}
