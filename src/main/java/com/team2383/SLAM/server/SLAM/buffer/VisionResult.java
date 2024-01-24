package com.team2383.SLAM.server.SLAM.buffer;

import edu.wpi.first.math.geometry.Transform3d;

public record VisionResult(Transform3d robotToTag, int landmarkIndex) {
}
