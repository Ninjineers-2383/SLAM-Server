package com.team2383.SLAM.server.SLAM;

import java.util.ArrayList;
import java.util.Iterator;

import com.team2383.SLAM.server.SLAM.Log.LogOutput;
import com.team2383.SLAM.server.SLAM.buffer.BufferEntry;
import com.team2383.SLAM.server.SLAM.buffer.TimeSyncedBuffer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class TimeSyncedSLAMLogger {
    private TimeSyncedBuffer buffer = new TimeSyncedBuffer();

    public void addEntry(BufferEntry entry) {
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
            while (next.isVisionEntry()) {
                intermediateVisionEntries.add(next);
                next = iterator.next();
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
