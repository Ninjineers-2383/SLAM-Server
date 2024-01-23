package com.team2383.SLAM.server.SLAM.buffer;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.geometry.Transform3d;

public record EKFSLAMVisionResult(Transform3d robotToTag, SimpleMatrix cov, int landmarkIndex) {
}
