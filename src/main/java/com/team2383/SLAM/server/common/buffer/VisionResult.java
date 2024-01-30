package com.team2383.SLAM.server.common.buffer;

import edu.wpi.first.math.geometry.Transform3d;

public record VisionResult(Transform3d robotToTag, int landmarkIndex) {
}
