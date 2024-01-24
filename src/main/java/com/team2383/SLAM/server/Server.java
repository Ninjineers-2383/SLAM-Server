package com.team2383.SLAM.server;

import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.time.format.DateTimeFormatter;
import java.util.Calendar;
import java.util.List;

import org.ejml.simple.SimpleMatrix;

import com.team2383.SLAM.server.SLAM.TimeSyncedSLAMLogger;
import com.team2383.SLAM.server.SLAM.Log.LogOutput;
import com.team2383.SLAM.server.SLAM.buffer.BufferEntry;
import com.team2383.SLAM.server.vision.VisionIONorthstar;
import com.team2383.SLAM.server.vision.VisionSubsystem;
import com.team2383.SLAM.server.vision.VisionSubsystem.TimestampVisionUpdate;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArraySubscriber;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.networktables.TimestampedObject;

public class Server {
    private VisionSubsystem m_visionSubsystem;
    private TimeSyncedSLAMLogger m_slam;

    private SimpleMatrix covariance;

    private int numLandmarks = 0;

    private StructSubscriber<Twist3d> twist3dSub;

    private final StructArraySubscriber<Transform3d> camTransformsSub;
    private final BooleanSubscriber saveAndExitSub;
    private final DoubleSubscriber varianceScaleSub;
    private final DoubleSubscriber varianceStaticSub;

    public Server() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("slam_data");

        twist3dSub = table.getStructTopic("twist3d", Twist3d.struct)
                .subscribe(new Twist3d(), PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));

        saveAndExitSub = table.getBooleanTopic("saveAndExit").subscribe(false);
        camTransformsSub = table.getStructArrayTopic("camTransforms", Transform3d.struct).subscribe(new Transform3d[0]);
        varianceScaleSub = table.getDoubleTopic("varianceScale").subscribe(0);
        varianceStaticSub = table.getDoubleTopic("varianceStatic").subscribe(0);

        reinitializeSLAM(numLandmarks, new Pose3d[0]);

        covariance = SimpleMatrix.identity(7).scale(0.01);
    }

    public void loop() {
        TimestampedObject<Twist3d>[] twists = twist3dSub.readQueue();
        m_visionSubsystem.setVisionConstants(camTransformsSub.get(), varianceScaleSub.get(), varianceStaticSub.get());

        for (TimestampedObject<Twist3d> twist : twists) {
            m_slam.addEntry(new BufferEntry(twist.value, covariance.copy(), twist.timestamp));
        }

        m_visionSubsystem.periodic();

        if (saveAndExitSub.get()) {
            System.out.println("Saving and exiting");
            LogOutput log = m_slam.export();
            log.saveg2o("output.g2o");
            System.exit(0);
        }
    }

    private void reinitializeSLAM(int numLandmarks, Pose3d[] landmarks) {

        m_visionSubsystem = new VisionSubsystem(
                landmarks,
                new VisionIONorthstar("northstar-1"),
                new VisionIONorthstar("northstar-2"),
                new VisionIONorthstar("northstar-3"),
                new VisionIONorthstar("northstar-4"));

        m_visionSubsystem.setVisionConsumer(this::visionConsumer);

        m_slam = new TimeSyncedSLAMLogger();
    }

    public void visionConsumer(List<TimestampVisionUpdate> visionUpdates) {
        for (TimestampVisionUpdate update : visionUpdates) {
            m_slam.addEntry(new BufferEntry(update.pose(), update.covariance(), update.tagId(), update.timestamp()));
        }
    }
}
