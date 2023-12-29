package team2383.SLAM;

import org.junit.jupiter.api.Test;

import com.team2383.SLAM.server.SLAM.buffer.EKFSLAMBuffer;
import com.team2383.SLAM.server.SLAM.buffer.EKFSLAMBufferEntry;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class EKFSLAMBufferTest {

    @Test
    public void testVisionEntryOnEmptyBuffer() {
        EKFSLAMBuffer buffer = new EKFSLAMBuffer();

        buffer.addVisionEntry(createVisionEntry(0));

        assert(buffer.isEmpty());
    }   

    @Test
    public void testOldVisionEntry() {
        EKFSLAMBuffer buffer = new EKFSLAMBuffer();

        buffer.addSpeedsEntry(createSpeedsEntry(1));
        buffer.addVisionEntry(createVisionEntry(0));

        assert(buffer.size() == 1);
    }

    @Test
    public void testNewVisionEntry() {
        EKFSLAMBuffer buffer = new EKFSLAMBuffer();

        buffer.addSpeedsEntry(createSpeedsEntry(0));
        buffer.addVisionEntry(createVisionEntry(1));

        assert(buffer.get(0).timestamp == 0);
        assert(buffer.get(1).timestamp == 1);
    }

    @Test
    public void testVisionEntryPlacement() {
        EKFSLAMBuffer buffer = new EKFSLAMBuffer();

        buffer.addSpeedsEntry(createSpeedsEntry(0));
        buffer.addVisionEntry(createVisionEntry(1));
        buffer.addSpeedsEntry(createSpeedsEntry(3));
        buffer.addVisionEntry(createVisionEntry(2));

        assert(buffer.get(0).timestamp == 0);
        assert(buffer.get(1).timestamp == 1);
        assert(buffer.get(2).timestamp == 2);
        assert(buffer.get(3).timestamp == 3);
    }

    @Test
    public void testOldSpeedsEntry() {
        EKFSLAMBuffer buffer = new EKFSLAMBuffer();

        buffer.addSpeedsEntry(createSpeedsEntry(2));
        buffer.addSpeedsEntry(createVisionEntry(1));

        assert(buffer.size() == 1);
    }

    @Test
    public void testOldSecondSpeedsEntry() {
        EKFSLAMBuffer buffer = new EKFSLAMBuffer();

        buffer.addSpeedsEntry(createSpeedsEntry(0));
        buffer.addSpeedsEntry(createSpeedsEntry(2));
        buffer.addSpeedsEntry(createSpeedsEntry(1));

        assert(buffer.size() == 2);
    }

    @Test
    public void testIntermideateSpeedsEntry() {
        EKFSLAMBuffer buffer = new EKFSLAMBuffer();

        buffer.addSpeedsEntry(createSpeedsEntry(0));
        buffer.addVisionEntry(createVisionEntry(2));
        buffer.addSpeedsEntry(createSpeedsEntry(1));

        assert(buffer.get(0).timestamp == 0);
        assert(buffer.get(1).timestamp == 1);
        assert(buffer.get(2).timestamp == 2);
    }

    @Test
    public void testRefreshBuffer() {
        EKFSLAMBuffer buffer = new EKFSLAMBuffer();

        buffer.addSpeedsEntry(createSpeedsEntry(0));
        buffer.addSpeedsEntry(createSpeedsEntry(1));
        buffer.addSpeedsEntry(createSpeedsEntry(2));

        assert(buffer.size() == 2);
        assert(buffer.get(0).timestamp == 1);
        assert(buffer.get(1).timestamp == 2);
    }

    public EKFSLAMBufferEntry createVisionEntry(double timestamp) {
        return new EKFSLAMBufferEntry(new Transform3d(), 0, timestamp);
    }

    public EKFSLAMBufferEntry createSpeedsEntry(double timestamp) {
        return new EKFSLAMBufferEntry(new ChassisSpeeds(), timestamp);
    }
}
