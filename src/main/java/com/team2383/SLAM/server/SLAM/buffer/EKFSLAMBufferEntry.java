package com.team2383.SLAM.server.SLAM.buffer;

import java.util.Optional;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class EKFSLAMBufferEntry {
        public SimpleMatrix mu;
        public SimpleMatrix sigma;
        public Optional<ChassisSpeeds> speeds;
        public Optional<Transform3d> robotToTag;
        public Optional<Integer> landmarkIndex;
        public double timestamp;

        public EKFSLAMBufferEntry(SimpleMatrix mu, SimpleMatrix sigma, ChassisSpeeds speeds, double timestamp) {
            this.mu = mu;
            this.sigma = sigma;
            this.speeds = Optional.of(speeds);
            this.robotToTag = Optional.empty();
            this.landmarkIndex = Optional.empty();
            this.timestamp = timestamp;
        }

        public EKFSLAMBufferEntry(SimpleMatrix mu, SimpleMatrix sigma, 
            Transform3d robotToTag, Integer landmarkIndex, double timestamp) {
            this.mu = mu;
            this.sigma = sigma;
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
    }