package team2383.SLAM;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import com.team2383.SLAM.server.SLAM.buffer.EKFSLAMBuffer;
import com.team2383.SLAM.server.SLAM.buffer.EKFSLAMBufferEntry;
import com.team2383.SLAM.server.SLAM.buffer.EKFSLAMState;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class EKFSLAMBufferTest {

    @Test
    public void testVisionEntryOnEmptyBuffer() {
        EKFSLAMBuffer buffer = new EKFSLAMBuffer((entry) -> {
            assert (entry.isVisionEntry());

            return new EKFSLAMState(null, null);
        });

        buffer.addVisionEntry(createVisionEntry(0));

        assertEquals(2, buffer.size());
        assertEquals(0, buffer.get(0).timestamp);
        assertEquals(0, buffer.get(1).timestamp);
        assert (buffer.get(0).isSpeedsEntry());
        assert (buffer.get(1).isVisionEntry());
        assert (buffer.get(0).state.isPresent());
        assert (!buffer.get(1).state.isPresent());
    }

    @Test
    public void testInitialSpeedsEntry() {
        EKFSLAMBuffer buffer = new EKFSLAMBuffer(this::createInitialState);

        buffer.addSpeedsEntry(createSpeedsEntry(1));
        buffer.addVisionEntry(createVisionEntry(0));

        assertEquals(2, buffer.size());
        assertEquals(0, buffer.get(0).timestamp);
        assertEquals(0, buffer.get(1).timestamp);
        assert (buffer.get(0).isSpeedsEntry());
        assert (buffer.get(1).isVisionEntry());
        assert (buffer.get(0).state.isPresent());
        assert (!buffer.get(1).state.isPresent());
    }

    @Test
    public void testNewVisionEntry() {
        EKFSLAMBuffer buffer = new EKFSLAMBuffer(this::createInitialState);

        buffer.addVisionEntry(createVisionEntry(0));

        buffer.addVisionEntry(createVisionEntry(1));

        assertEquals(0, buffer.get(0).timestamp);
        assert (buffer.get(0).isSpeedsEntry());
        assertEquals(0, buffer.get(1).timestamp);
        assert (buffer.get(1).isVisionEntry());
        assertEquals(1, buffer.get(2).timestamp);
        assert (buffer.get(2).isVisionEntry());
    }

    @Test
    public void testVisionEntryPlacement() {
        EKFSLAMBuffer buffer = new EKFSLAMBuffer(this::createInitialState);

        buffer.addVisionEntry(createVisionEntry(1));
        buffer.addSpeedsEntry(createSpeedsEntry(3));
        buffer.addVisionEntry(createVisionEntry(2));

        assertEquals(1, buffer.get(0).timestamp);
        assert (buffer.get(0).isSpeedsEntry());
        assertEquals(1, buffer.get(1).timestamp);
        assert (buffer.get(1).isVisionEntry());
        assertEquals(2, buffer.get(2).timestamp);
        assert (buffer.get(2).isVisionEntry());
        assertEquals(3, buffer.get(3).timestamp);
        assert (buffer.get(3).isSpeedsEntry());
    }

    @Test
    public void testOldSpeedsEntry() {
        EKFSLAMBuffer buffer = new EKFSLAMBuffer(this::createInitialState);

        buffer.addVisionEntry(createVisionEntry(2));
        buffer.addSpeedsEntry(createVisionEntry(1));

        assertEquals(2, buffer.size());
        assertEquals(2, buffer.get(0).timestamp);
        assert(buffer.get(0).isSpeedsEntry());
        assertEquals(2, buffer.get(1).timestamp);
        assert(buffer.get(1).isVisionEntry());
    }

    @Test
    public void testOldSecondSpeedsEntry() {
        EKFSLAMBuffer buffer = new EKFSLAMBuffer(this::createInitialState);

        buffer.addVisionEntry(createVisionEntry(0));

        buffer.addSpeedsEntry(createSpeedsEntry(2));

        buffer.addSpeedsEntry(createSpeedsEntry(1));

        assertEquals(3, buffer.size());
    }

    @Test
    public void testIntermediateSpeedsEntry() {
        EKFSLAMBuffer buffer = new EKFSLAMBuffer(this::createInitialState);

        buffer.addVisionEntry(createVisionEntry(0));
        buffer.addVisionEntry(createVisionEntry(2));
        buffer.addSpeedsEntry(createSpeedsEntry(1));

        assertEquals(0, buffer.get(0).timestamp);
        assertEquals(0, buffer.get(1).timestamp);
        assertEquals(1, buffer.get(2).timestamp);
        assertEquals(2, buffer.get(3).timestamp);
    }

    @Test
    public void testClearBuffer() {
        EKFSLAMBuffer buffer = new EKFSLAMBuffer(this::createInitialState);

        buffer.addVisionEntry(createVisionEntry(0));
        buffer.addSpeedsEntry(createSpeedsEntry(1));
        buffer.addSpeedsEntry(createSpeedsEntry(2));

        assertEquals(2, buffer.size());

        assertEquals(1, buffer.get(0).timestamp);
        assertEquals(2, buffer.get(1).timestamp);
    }

    @Test 
    public void testSecondSpeedsIndex() {
        EKFSLAMBuffer buffer = new EKFSLAMBuffer(this::createInitialState);

        buffer.addSpeedsEntry(createSpeedsEntry(0));
        buffer.addVisionEntry(createVisionEntry(1));
        buffer.addSpeedsEntry(createSpeedsEntry(4));

        assertEquals(2, buffer.getSecondSpeedsIndex());

        buffer.addVisionEntry(createVisionEntry(2));
        
        assertEquals(3, buffer.getSecondSpeedsIndex());

        buffer.addVisionEntry(createVisionEntry(5));

        assertEquals(3, buffer.getSecondSpeedsIndex());
    }

    @Test
    public void testHasSecondSpeedsEntry() {
        EKFSLAMBuffer buffer = new EKFSLAMBuffer(this::createInitialState);

        buffer.addSpeedsEntry(createSpeedsEntry(0));
        buffer.addVisionEntry(createVisionEntry(1));
        buffer.addSpeedsEntry(createSpeedsEntry(4));

        assert (buffer.hasSecondSpeedsEntry());
    }

    public EKFSLAMBufferEntry createVisionEntry(double timestamp) {
        return new EKFSLAMBufferEntry(new Transform3d(), 0, timestamp);
    }

    public EKFSLAMBufferEntry createSpeedsEntry(double timestamp) {
        return new EKFSLAMBufferEntry(new ChassisSpeeds(), timestamp);
    }

    private EKFSLAMState createInitialState(EKFSLAMBufferEntry entry) {
        return new EKFSLAMState(null, null);
    }
}
