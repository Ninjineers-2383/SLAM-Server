package com.team2383.SLAM.server.SLAM.buffer;

import java.util.Optional;

import org.ejml.simple.SimpleMatrix;

import com.team2383.SLAM.server.TimedChassisSpeeds;

import edu.wpi.first.math.geometry.Transform3d;

public class EKFSLAMBufferEntry {
    public final Optional<EKFSLAMChassisResult> speeds;
    public final Optional<EKFSLAMVisionResult> visionResult;
    public final double timestamp;
    public Optional<EKFSLAMState> state;

    public EKFSLAMBufferEntry(TimedChassisSpeeds speeds, double timestamp) {
        this.state = Optional.empty();
        this.speeds = Optional.of(new EKFSLAMChassisResult(speeds));
        this.visionResult = Optional.empty();
        this.timestamp = timestamp;
    }

    public EKFSLAMBufferEntry(Transform3d robotToTag, SimpleMatrix cov, Integer landmarkIndex, double timestamp) {
        this.state = Optional.empty();
        this.speeds = Optional.empty();
        this.visionResult = Optional.of(new EKFSLAMVisionResult(robotToTag, cov, landmarkIndex));
        this.timestamp = timestamp;
    }

    public boolean isSpeedsEntry() {
        return speeds.isPresent();
    }

    public boolean isVisionEntry() {
        return visionResult.isPresent();
    }

    public void updateMuAndSigma(SimpleMatrix mu, SimpleMatrix sigma) {
        this.state = Optional.of(new EKFSLAMState(mu, sigma));
    }

    public void updateMuAndSigma(EKFSLAMState state) {
        this.state = Optional.of(state);
    }
}
