package com.team2383.SLAM.server.SLAM;

import java.util.ArrayList;
import java.util.Iterator;

import com.team2383.SLAM.server.ISLAMProvider;
import com.team2383.SLAM.server.SLAM.Log.LogOutput;
import com.team2383.SLAM.server.common.buffer.BufferEntry;
import com.team2383.SLAM.server.common.buffer.TimeSyncedBuffer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;

public class TimeSyncedSLAMLogger implements ISLAMProvider {
    private final TimeSyncedBuffer buffer = new TimeSyncedBuffer();
    private final SwerveDriveKinematics kinematics;

    public TimeSyncedSLAMLogger(SwerveDriveKinematics kinematics) {
        this.kinematics = kinematics;
    }

    @Override
    public void addEntry(BufferEntry entry) {
        // System.out.println("Added entry" + entry.toString());
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
        while (!chassisTwo.isSpeedsEntry()) {
            chassisTwo = iterator.next();
        }
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

            SwerveDriveWheelPositions wheelPositionsOne = new SwerveDriveWheelPositions(
                    chassisOne.robot.get().update().modulePositions);
            SwerveDriveWheelPositions wheelPositionsTwo = new SwerveDriveWheelPositions(
                    chassisTwo.robot.get().update().modulePositions);

            Twist2d twist = kinematics.toTwist2d(wheelPositionsOne, wheelPositionsTwo);

            Rotation3d rotationOne = chassisOne.robot.get().update().gyroAngle;
            Rotation3d rotationTwo = chassisTwo.robot.get().update().gyroAngle;

            Rotation3d delta = rotationTwo.minus(rotationOne);

            Twist3d twist3d = new Twist3d(twist.dx, twist.dy, 0, delta.getX(), delta.getY(), delta.getZ());

            Pose3d poseTwo = poseOne.exp(twist3d);

            for (BufferEntry bufferEntry : intermediateVisionEntries) {
                double timeToCurrent = bufferEntry.timestamp - chassisOne.timestamp;

                SwerveDriveWheelPositions positions = wheelPositionsOne.interpolate(wheelPositionsTwo,
                        timeToCurrent / timeBetween);

                Rotation3d rotation = rotationOne.interpolate(rotationTwo, timeToCurrent / timeBetween);

                Rotation3d delPartialRot = rotation.minus(rotationOne);

                Twist2d partial = kinematics.toTwist2d(wheelPositionsOne, positions);

                Twist3d partial3d = new Twist3d(partial.dx, partial.dy, 0, delPartialRot.getX(), delPartialRot.getY(),
                        delPartialRot.getZ());

                Pose3d partialPose = poseOne.exp(partial3d);

                Transform3d visionTransform = bufferEntry.vision.get().robotToTag();

                int id = log.addChassisState(partialPose, chassisTwo.cov.invert());
                log.addVisionEdge(id, bufferEntry.vision.get().landmarkIndex1(),
                        visionTransform,
                        bufferEntry.cov.invert());
            }

            log.addChassisState(poseTwo, chassisTwo.cov.invert());
        }

        return log;
    }

    @Override
    public Pose3d getLatestPose() {
        return null;
    }

    @Override
    public double getLatestPoseTime() {
        return 0;
    }
}
