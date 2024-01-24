package com.team2383.SLAM.server.SLAM.Log;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.geometry.Transform3d;

public record LogEdge(Integer from, Integer to, Transform3d distance, SimpleMatrix information) {

}
