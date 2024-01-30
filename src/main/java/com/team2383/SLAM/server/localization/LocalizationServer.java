package com.team2383.SLAM.server.localization;

import com.team2383.SLAM.server.ISLAMProvider;
import com.team2383.SLAM.server.common.buffer.BufferEntry;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class LocalizationServer implements ISLAMProvider {
    private final SwerveDrivePoseEstimator estimator;
    private final Pose3d[] landmarks;

    private double latestTime = 0;

    public LocalizationServer(Pose3d[] landmarks, SwerveDriveKinematics kinematics) {
        estimator = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(), new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
        },
                new Pose2d());
        this.landmarks = landmarks;
    }

    @Override
    public void addEntry(BufferEntry entry) {
        if (entry.isVisionEntry()) {
            Pose2d pose = landmarks[entry.vision.get().landmarkIndex() - 1]
                    .plus(entry.vision.get().robotToTag().inverse())
                    .toPose2d();
            estimator.addVisionMeasurement(pose, entry.timestamp, VecBuilder.fill(0.1, 0.1, 0.1));
        } else {
            estimator.updateWithTime(entry.timestamp, entry.robot.get().update().gyroAngle.toRotation2d(),
                    entry.robot.get().update().modulePositions);
            latestTime = entry.robot.get().update().timestamp;
        }
    }

    public Pose3d getPose() {
        Pose2d pose = estimator.getEstimatedPosition();
        return new Pose3d(pose.getX(), pose.getY(), 0, new Rotation3d(0, 0, pose.getRotation().getRadians()));
    }

    @Override
    public Pose3d getLatestPose() {
        return getPose();
    }

    @Override
    public double getLatestPoseTime() {
        return latestTime;
    }
}
