package com.team2383.SLAM.server.SLAM;

import java.util.ArrayList;
import java.util.Iterator;

import com.team2383.SLAM.server.SLAM.Log.LogOutput;
import com.team2383.SLAM.server.SLAM.buffer.BufferEntry;
import com.team2383.SLAM.server.SLAM.buffer.TimeSyncedBuffer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Twist3d;

public class TimeSyncedSLAMLogger {
    private TimeSyncedBuffer buffer = new TimeSyncedBuffer();

    boolean enabled = true;

    private double getTwistSqNorm(Twist3d twist3d) {
        return twist3d.dx * twist3d.dx + twist3d.dy * twist3d.dy
                + twist3d.dz * twist3d.dz;
    }

    public void addEntry(BufferEntry entry) {
        if (!enabled) {
            if (entry.isSpeedsEntry() && getTwistSqNorm(entry.chassis.get().twist3d()) > 1E-5) {
                enabled = true;
            } else {
                return;
            }
        }
        buffer.addEntry(entry);
    }

    private Iterator<BufferEntry> getEntries() {
        return buffer.getIterator();
    }

    public LogOutput export() {
        LogOutput log = new LogOutput();

        Iterator<BufferEntry> iterator = getEntries();
        BufferEntry chassisOne;
        BufferEntry chassisTwo = iterator.next();
        ArrayList<BufferEntry> intermediateVisionEntries = new ArrayList<>();

        log.addChassisState(new Pose3d(), null);

        while (iterator.hasNext()) {
            chassisOne = chassisTwo;
            intermediateVisionEntries.clear();
            BufferEntry next = iterator.next();
            while (next.isVisionEntry() && iterator.hasNext()) {
                intermediateVisionEntries.add(next);
                next = iterator.next();
            }
            if (next.isVisionEntry()) {
                break;
            }
            chassisTwo = next;

            double timeBetween = chassisTwo.timestamp - chassisOne.timestamp;

            Pose3d poseOne = log.getLatestPose();

            Pose3d poseTwo = poseOne.exp(chassisTwo.chassis.get().twist3d());

            for (BufferEntry bufferEntry : intermediateVisionEntries) {
                double timeToCurrent = bufferEntry.timestamp - chassisOne.timestamp;
                Pose3d partial = poseOne.interpolate(poseTwo, timeToCurrent / timeBetween);

                Transform3d visionTransform = bufferEntry.vision.get().robotToTag();

                int id = log.addChassisState(partial, chassisTwo.cov.invert());
                log.addVisionEdge(id, bufferEntry.vision.get().landmarkIndex(),
                        visionTransform,
                        bufferEntry.cov.invert());
            }

            log.addChassisState(poseTwo, chassisTwo.cov.invert());
        }

        return log;
    }
}
