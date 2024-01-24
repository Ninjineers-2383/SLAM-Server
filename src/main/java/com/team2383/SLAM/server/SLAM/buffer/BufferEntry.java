package com.team2383.SLAM.server.SLAM.buffer;

import java.util.Optional;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Twist3d;

public class BufferEntry implements Comparable<BufferEntry> {
    public final Optional<ChassisResults> chassis;
    public final Optional<VisionResult> vision;
    public final SimpleMatrix cov;
    public final double timestamp;

    public BufferEntry(Twist3d twist3d, SimpleMatrix cov, double timestamp) {
        this.chassis = Optional.of(new ChassisResults(twist3d));
        this.vision = Optional.empty();
        this.cov = cov;
        this.timestamp = timestamp;
    }

    public BufferEntry(Transform3d robotToTag, SimpleMatrix cov, Integer landmarkIndex, double timestamp) {
        this.chassis = Optional.empty();
        this.vision = Optional.of(new VisionResult(robotToTag, landmarkIndex));
        this.cov = cov;
        this.timestamp = timestamp;
    }

    public boolean isSpeedsEntry() {
        return chassis.isPresent();
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
}
