package team2383.SLAM;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.team2383.SLAM.server.SLAM.TimeSyncedEKFSLAM;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class TimeSyncedEKFSLAMTest {
    private final Pose3d[] landmarkSeed = new Pose3d[] {
            new Pose3d(),
            new Pose3d(1, 0, 0, new Rotation3d()),
            new Pose3d(0, 1, 0, new Rotation3d()),
            new Pose3d(0, 0, 1, new Rotation3d()),
    };

    @Test
    public void testInitialVisionEntry() {
        TimeSyncedEKFSLAM SLAM = new TimeSyncedEKFSLAM(landmarkSeed.length, landmarkSeed);

        SLAM.addVisionMeasurement(new Transform3d(), 0, 0);

        assertEquals(new Pose3d(), SLAM.getRobotPose());
    }

    @Test
    public void testChunkAtOrigin() {
        TimeSyncedEKFSLAM SLAM = new TimeSyncedEKFSLAM(landmarkSeed.length, landmarkSeed);

        SLAM.addVisionMeasurement(new Transform3d(), 0, 0);
        SLAM.addDriveOdometryMeasurement(new ChassisSpeeds(), 1);

        assertEquals(new Pose3d(), SLAM.getRobotPose());
    }

    @Test
    public void testChunkNotAtOrigin() {
        TimeSyncedEKFSLAM SLAM = new TimeSyncedEKFSLAM(landmarkSeed.length, landmarkSeed);

        SLAM.addVisionMeasurement(new Transform3d(1, 0, 0, new Rotation3d()), 1, 0);
        SLAM.addDriveOdometryMeasurement(new ChassisSpeeds(), 1);

        assertEquals(new Pose3d(), SLAM.getRobotPose());
    }

    @Test
    public void testRepeatedEmptyChassisSpeeds() {
        TimeSyncedEKFSLAM SLAM = new TimeSyncedEKFSLAM(landmarkSeed.length, landmarkSeed);

        SLAM.addVisionMeasurement(new Transform3d(), 0, 0);
        SLAM.addDriveOdometryMeasurement(new ChassisSpeeds(), 1);
        SLAM.addDriveOdometryMeasurement(new ChassisSpeeds(), 2);
        SLAM.addDriveOdometryMeasurement(new ChassisSpeeds(), 3);
        SLAM.addDriveOdometryMeasurement(new ChassisSpeeds(), 4);

        assertEquals(new Pose3d(), SLAM.getRobotPose());
    }

    @Test
    public void testRepeatedVisionMeasurements() {
        TimeSyncedEKFSLAM SLAM = new TimeSyncedEKFSLAM(landmarkSeed.length, landmarkSeed);

        SLAM.addVisionMeasurement(new Transform3d(), 0, 0);
        SLAM.addDriveOdometryMeasurement(new ChassisSpeeds(), 1);
        SLAM.addVisionMeasurement(new Transform3d(), 0, 3);
        SLAM.addVisionMeasurement(new Transform3d(), 0, 2);
        SLAM.addVisionMeasurement(new Transform3d(1, 0, 0, new Rotation3d()), 1, 1);

        assertEquals(new Pose3d(), SLAM.getRobotPose());
    }
}
