package com.team2383.SLAM.server.SLAM.buffer;

import edu.wpi.first.math.geometry.Transform3d;

public record EKFSLAMVisionResult(Transform3d robotToTag, int landmarkIndex) {
}
