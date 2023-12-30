package com.team2383.SLAM.server.SLAM.buffer;

import org.ejml.simple.SimpleMatrix;

public record EKFSLAMState(SimpleMatrix mu, SimpleMatrix sigma) {
}
