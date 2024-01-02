package team2383.SLAM;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.team2383.SLAM.server.SLAM.TimeSyncedEKFSLAM;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
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
        SLAM.addDriveOdometryMeasurement(new ChassisSpeeds(), 4);
        SLAM.addVisionMeasurement(new Transform3d(1, 0, 0, new Rotation3d()), 1, 5);
        SLAM.addVisionMeasurement(new Transform3d(1, 0, 0, new Rotation3d()), 1, 6);
        SLAM.addDriveOdometryMeasurement(new ChassisSpeeds(), 7);

        assertEquals(new Pose3d(), SLAM.getRobotPose());
    }

    @Test
    public void slightlyOffAxis() {
        Pose3d delta = new Pose3d(0.1, 0.1, 0.1, new Rotation3d(new Quaternion(0.4, 0.0, 0.0, 0.4)));
        TimeSyncedEKFSLAM SLAM = new TimeSyncedEKFSLAM(landmarkSeed.length, landmarkSeed);

        SLAM.addVisionMeasurement(new Transform3d(0, 0, 0,
                new Rotation3d(new Quaternion(0.9842372831972948, 0, 0, -0.17685296255479674))), 0, 0);
        SLAM.addDriveOdometryMeasurement(new ChassisSpeeds(), 0.1);

        for (int i = 1; i <= 100; i++) {
            SLAM.addVisionMeasurement(new Transform3d(), 0, i);
            SLAM.addDriveOdometryMeasurement(new ChassisSpeeds(), i + 0.1);
            assertTrue(deltaEquals(new Pose3d(), SLAM.getRobotPose(), delta),
                    String.format("Expected %s to be within %s of %s at iteration %d", SLAM.getRobotPose(),
                            delta,
                            new Pose3d(), i));
        }

        SLAM.addDriveOdometryMeasurement(new ChassisSpeeds(), 101);

        assertTrue(deltaEquals(new Pose3d(), SLAM.getRobotPose(), delta));
    }

    private boolean deltaEquals(Pose3d a, Pose3d b, Pose3d delta) {
        boolean x = Math.abs(a.getTranslation().getX() - b.getTranslation().getX()) < delta.getTranslation().getX();
        boolean y = Math.abs(a.getTranslation().getY() - b.getTranslation().getY()) < delta.getTranslation().getY();
        boolean z = Math.abs(a.getTranslation().getZ() - b.getTranslation().getZ()) < delta.getTranslation().getZ();
        boolean qw = Math.abs(a.getRotation().getQuaternion().getW() - b.getRotation().getQuaternion().getW()) < (delta
                .getRotation().getQuaternion().getW() + 0.0001);
        boolean qx = Math.abs(a.getRotation().getQuaternion().getX() - b.getRotation().getQuaternion().getX()) < (delta
                .getRotation().getQuaternion().getX() + 0.0001);
        boolean qy = Math.abs(a.getRotation().getQuaternion().getY() - b.getRotation().getQuaternion().getY()) < (delta
                .getRotation().getQuaternion().getY() + 0.0001);
        boolean qz = Math.abs(a.getRotation().getQuaternion().getZ() - b.getRotation().getQuaternion().getZ()) < (delta
                .getRotation().getQuaternion().getZ() + 0.0001);

        return x && y && z && qw && qx && qy && qz;
    }
}
