package com.team2383.SLAM.server.common;

import edu.wpi.first.math.geometry.Transform3d;

public record TagDetectionState(Transform3d transform, int tag, double timestamp) {

}
