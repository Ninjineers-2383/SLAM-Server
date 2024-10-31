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

    private BufferEntry latestChassisState;

    private PoseFilter poseFilter;
    private double latestTime = 0;

    private long seenLandmarks;

    private int poseCount = 0;

    public LocalizationServer(Pose3d[] landmarks, SwerveDriveKinematics kinematics, int poseFilterSize,
            double poseFilterOutlierDistance) {
        estimator = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(), new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
        },
                new Pose2d());
        this.landmarks = landmarks;

        poseFilter = new PoseFilter(poseFilterSize, poseFilterOutlierDistance);
        seenLandmarks = 0;
    }

    @Override
    public void addEntry(BufferEntry entry) {
        if (entry.isVisionEntry()) {
            Pose2d pose = landmarks[entry.vision.get().landmarkIndex1() - 1]
                    .plus(entry.vision.get().robotToTag().inverse())
                    .toPose2d();
            seenLandmarks |= getTagBitmask(
                    entry.vision.get().landmarkIndex1(),
                    entry.vision.get().landmarkIndex2(),
                    entry.vision.get().cameraId());
            if (!poseFilter.next(pose)) {
                return;
            }
            estimator.addVisionMeasurement(pose, entry.timestamp,
                    VecBuilder.fill(entry.cov.get(0, 0), entry.cov.get(1, 1), entry.cov.get(5, 5)));
        } else {
            estimator.updateWithTime(entry.timestamp, entry.robot.get().update().gyroAngle.toRotation2d(),
                    entry.robot.get().update().modulePositions);
            latestTime = entry.robot.get().update().timestamp;

            latestChassisState = entry;
        }
    }

    private long getTagBitmask(int landmark1, int landmark2, int cameraIndex) {
        long value = 0;
        value |= 1 << (landmark1 - 1) << (cameraIndex * 16);
        if (landmark2 != 0) {
            value |= (1 << (landmark2 - 1)) << (cameraIndex * 16);
        }
        return value;
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

    public void setOutlierRejectionDistance(double distance) {
        poseFilter.rejectionDistance = distance;
    }

    public void resetPose(Pose2d pose) {
        estimator.resetPosition(latestChassisState.robot.get().update().gyroAngle.toRotation2d(),
                latestChassisState.robot.get().update().modulePositions, pose);
    }

    public long getSeenLandmarks() {
        long copy = seenLandmarks;
        poseCount++;
        if (poseCount == 25)
            seenLandmarks = 0;
        return copy;
    }
}
