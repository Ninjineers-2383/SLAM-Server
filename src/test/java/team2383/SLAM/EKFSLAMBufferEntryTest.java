package team2383.SLAM;

import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Test;

import com.team2383.SLAM.server.SLAM.buffer.EKFSLAMBufferEntry;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class EKFSLAMBufferEntryTest {

    @Test
    public void testSpeedsEntry() {
        EKFSLAMBufferEntry entry = new EKFSLAMBufferEntry(SimpleMatrix.identity(5), SimpleMatrix.identity(5), new ChassisSpeeds(), 0);

        assert(!entry.isVisionEntry());
        assert(entry.isSpeedsEntry());
    }

    @Test
    public void testVisionEntry() {
        EKFSLAMBufferEntry entry = new EKFSLAMBufferEntry(SimpleMatrix.identity(5), SimpleMatrix.identity(5), new Transform3d(), 1, 0);

        assert(entry.isVisionEntry());
        assert(!entry.isSpeedsEntry());
    }
}
