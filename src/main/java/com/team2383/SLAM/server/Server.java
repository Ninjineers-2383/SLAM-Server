package com.team2383.SLAM.server;

import java.util.List;

import org.ejml.simple.SimpleMatrix;

import com.team2383.SLAM.server.SLAM.TimeSyncedSLAMLogger;
import com.team2383.SLAM.server.SLAM.Log.LogOutput;
import com.team2383.SLAM.server.common.buffer.BufferEntry;
import com.team2383.SLAM.server.localization.LocalizationServer;
import com.team2383.SLAM.server.timedTypes.TimedPose3d;
import com.team2383.SLAM.server.timedTypes.TimedRobotUpdate;
import com.team2383.SLAM.server.vision.VisionIONorthstar;
import com.team2383.SLAM.server.vision.VisionSubsystem;
import com.team2383.SLAM.server.vision.VisionSubsystem.TimestampVisionUpdate;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArraySubscriber;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.networktables.TimestampedObject;

public class Server {
    private VisionSubsystem m_visionSubsystem;
    private ISLAMProvider m_slam;

    private SimpleMatrix covariance;

    private final StructArraySubscriber<Pose3d> landmarksSub;

    private final StructSubscriber<TimedRobotUpdate> robotUpdateSub;

    private final StructArraySubscriber<Translation2d> moduleLocationsSub;

    private final StructArraySubscriber<Transform3d> camTransformsSub;
    private final BooleanSubscriber saveAndExitSub;
    private final DoubleSubscriber varianceScaleSub;
    private final DoubleSubscriber varianceStaticSub;

    private final StructPublisher<TimedPose3d> posePub;

    private Pose3d[] landmarks = new Pose3d[0];
    private Translation2d[] moduleLocations;

    public Server() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("slam_data");

        saveAndExitSub = table.getBooleanTopic("saveAndExit").subscribe(false);
        camTransformsSub = table.getStructArrayTopic("camTransforms", Transform3d.struct).subscribe(new Transform3d[0]);
        varianceScaleSub = table.getDoubleTopic("varianceScale").subscribe(0);
        varianceStaticSub = table.getDoubleTopic("varianceStatic").subscribe(0);

        robotUpdateSub = table.getStructTopic("robotUpdate", TimedRobotUpdate.struct)
                .subscribe(new TimedRobotUpdate(), PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));

        moduleLocationsSub = table.getStructArrayTopic("moduleLocations", Translation2d.struct)
                .subscribe(new Translation2d[0], PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true),
                        PubSubOption.pollStorage(1), PubSubOption.keepDuplicates(true));

        landmarksSub = table.getStructArrayTopic("landmarks", Pose3d.struct).subscribe(new Pose3d[0],
                PubSubOption.pollStorage(1), PubSubOption.keepDuplicates(true));

        posePub = table.getStructTopic("pose", TimedPose3d.struct).publish(PubSubOption.sendAll(true),
                PubSubOption.keepDuplicates(false));

        covariance = SimpleMatrix.diag(1, 1, 0.001, 0.001, 0.001, 1).scale(0.01);
    }

    public void loop() {
        TimestampedObject<Translation2d[]>[] modulePositions = moduleLocationsSub.readQueue();
        TimestampedObject<Pose3d[]>[] landmarks = landmarksSub.readQueue();

        if (modulePositions.length > 0 || landmarks.length > 0) {
            if (landmarks.length > 0) {
                this.landmarks = landmarks[0].value;
            }
            if (modulePositions.length > 0) {
                this.moduleLocations = modulePositions[0].value;
            }
            if (this.moduleLocations == null || this.landmarks == null) {
                return;
            }
            if (moduleLocations.length != 4) {
                throw new IllegalArgumentException("modulePositions must be length 4");
            }
            reinitializeSLAM(this.landmarks, this.moduleLocations);
        }

        if (m_slam == null) {
            return;
        }

        TimestampedObject<TimedRobotUpdate>[] updates = robotUpdateSub.readQueue();
        m_visionSubsystem.setVisionConstants(camTransformsSub.get(), varianceScaleSub.get(), varianceStaticSub.get());

        for (TimestampedObject<TimedRobotUpdate> robotUpdate : updates) {
            m_slam.addEntry(new BufferEntry(robotUpdate.value, covariance.copy(), robotUpdate.serverTime / 1000000.0));
        }

        m_visionSubsystem.periodic();

        if (saveAndExitSub.get()) {
            if (m_slam instanceof TimeSyncedSLAMLogger) {
                System.out.println("Saving and exiting");
                LogOutput log = ((TimeSyncedSLAMLogger) m_slam).export();
                log.saveg2o("output.g2o");
                System.exit(0);
            }
        }

        posePub.set(new TimedPose3d(m_slam.getLatestPose(),
                m_slam.getLatestPoseTime()));
        // posePub.set(m_slam.getLatestPose());
    }

    private void reinitializeSLAM(Pose3d[] landmarks, Translation2d[] moduleLocations) {
        m_visionSubsystem = new VisionSubsystem(
                landmarks,
                new VisionIONorthstar("northstar-1"),
                new VisionIONorthstar("northstar-2"),
                new VisionIONorthstar("northstar-3"),
                new VisionIONorthstar("northstar-4"));

        m_visionSubsystem.setVisionConsumer(this::visionConsumer);

        if (landmarks.length == 0) {
            if (moduleLocations.length != 4) {
                throw new IllegalArgumentException("moduleLocations must be length 4");
            }
            m_slam = new TimeSyncedSLAMLogger(new SwerveDriveKinematics(moduleLocations));
        } else {
            m_slam = new LocalizationServer(landmarks, new SwerveDriveKinematics(moduleLocations));
        }

        System.out.println("SLAM reinitialized");
    }

    public void visionConsumer(List<TimestampVisionUpdate> visionUpdates) {
        for (TimestampVisionUpdate update : visionUpdates) {
            m_slam.addEntry(new BufferEntry(update.pose(), update.covariance(), update.tagId(), update.timestamp()));
        }
    }
}
