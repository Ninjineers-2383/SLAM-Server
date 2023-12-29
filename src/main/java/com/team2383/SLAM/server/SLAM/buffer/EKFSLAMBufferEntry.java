package com.team2383.SLAM.server.SLAM.buffer;

import java.util.Optional;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class EKFSLAMBufferEntry {
        public Optional<SimpleMatrix> mu;
        public Optional<SimpleMatrix> sigma;
        public final Optional<ChassisSpeeds> speeds;
        public final Optional<Transform3d> robotToTag;
        public final Optional<Integer> landmarkIndex;
        public final double timestamp;

        public EKFSLAMBufferEntry(ChassisSpeeds speeds, double timestamp) {
            this.mu = Optional.empty();
            this.sigma = Optional.empty();
            this.speeds = Optional.of(speeds);
            this.robotToTag = Optional.empty();
            this.landmarkIndex = Optional.empty();
            this.timestamp = timestamp;
        }

        public EKFSLAMBufferEntry(Transform3d robotToTag, Integer landmarkIndex, double timestamp) {
            this.mu = Optional.empty();
            this.sigma = Optional.empty();
            this.speeds = Optional.empty();
            this.robotToTag = Optional.of(robotToTag);
            this.landmarkIndex = Optional.of(landmarkIndex);
            this.timestamp = timestamp;
        }

        public boolean isSpeedsEntry() {
            return speeds.isPresent();
        }

        public boolean isVisionEntry() {
            return robotToTag.isPresent() && landmarkIndex.isPresent();
        }

        public void updateMuAndSigma(SimpleMatrix mu, SimpleMatrix sigma) {
            this.mu = Optional.of(mu);
            this.sigma = Optional.of(sigma);
        }
    }