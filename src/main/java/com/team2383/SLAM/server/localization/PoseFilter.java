package com.team2383.SLAM.server.localization;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PoseFilter {
    public final MedianFilter x;
    public final MedianFilter y;
    public double rejectionDistance;

    public PoseFilter(int size, double rejectionDistance) {
        x = new MedianFilter(size);
        y = new MedianFilter(size);
        this.rejectionDistance = rejectionDistance;
    }

    public boolean next(Pose2d pose) {
        double x_med = x.calculate(pose.getX());
        double y_med = x.calculate(pose.getY());

        Translation2d median = new Translation2d(x_med, y_med);

        return pose.getTranslation().getDistance(median) < rejectionDistance;
    }
}
