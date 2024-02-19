package com.team2383.SLAM.server.common.buffer;

import java.util.Optional;

import org.ejml.simple.SimpleMatrix;

import com.team2383.SLAM.server.timedTypes.TimedRobotUpdate;

import edu.wpi.first.math.geometry.Transform3d;

public class BufferEntry implements Comparable<BufferEntry> {
    public final Optional<RobotResult> robot;
    public final Optional<VisionResult> vision;
    public final SimpleMatrix cov;
    public final double timestamp;

    public BufferEntry(TimedRobotUpdate robot, SimpleMatrix cov, double timestamp) {
        this.robot = Optional.of(new RobotResult(robot));
        this.vision = Optional.empty();
        this.cov = cov;
        this.timestamp = timestamp;
    }

    public BufferEntry(Transform3d robotToTag, SimpleMatrix cov, Integer landmarkIndex, double timestamp) {
        this.robot = Optional.empty();
        this.vision = Optional.of(new VisionResult(robotToTag, landmarkIndex));
        this.cov = cov;
        this.timestamp = timestamp;
    }

    public boolean isSpeedsEntry() {
        return robot.isPresent();
    }

    public boolean isVisionEntry() {
        return vision.isPresent();
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof BufferEntry) {
            BufferEntry other = (BufferEntry) obj;
            return other.timestamp == this.timestamp;
        }
        return false;
    }

    @Override
    public int compareTo(BufferEntry o) {
        return Double.compare(this.timestamp, o.timestamp);
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("BufferEntry [timestamp=" + timestamp + ", ");
        if (isSpeedsEntry()) {
            sb.append("robot=");
            sb.append(robot.get().toString());
        } else if (isVisionEntry()) {
            sb.append("vision=");
            sb.append(vision.get().toString());
        } else {
            sb.append("Unknown");
        }
        sb.append("]");
        return sb.toString();
    }
}
