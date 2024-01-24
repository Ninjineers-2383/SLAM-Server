package com.team2383.SLAM.server.SLAM;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Twist3d;

public record RobotState(Pose3d pose, Twist3d twist, double timestamp) {

}
