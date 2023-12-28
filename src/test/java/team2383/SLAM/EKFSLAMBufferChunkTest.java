package team2383.SLAM;

import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Test;

import com.team2383.SLAM.server.SLAM.buffer.EKFSLAMBufferEntry;
import com.team2383.SLAM.server.SLAM.buffer.EKFSLAMBufferChunk;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class EKFSLAMBufferChunkTest {

    @Test
    public void testInitialVisionEntry() {
        EKFSLAMBufferChunk chunk = new EKFSLAMBufferChunk();

        EKFSLAMBufferEntry visionEntry = new EKFSLAMBufferEntry(SimpleMatrix.identity(5), SimpleMatrix.identity(5), new Transform3d(), 1, 1);

        chunk.addEntry(visionEntry);

        assert(chunk.isEmpty());
    }

    @Test
    public void testInitialSpeedsEntry() {
        EKFSLAMBufferChunk chunk = new EKFSLAMBufferChunk();

        EKFSLAMBufferEntry speedsEntry = new EKFSLAMBufferEntry(SimpleMatrix.identity(5), SimpleMatrix.identity(5), new ChassisSpeeds(), 1);

        chunk.addEntry(speedsEntry);

        assert(!chunk.isEmpty());
    }

    @Test
    public void testCompleteChunk() {
        EKFSLAMBufferChunk chunk = new EKFSLAMBufferChunk();

        EKFSLAMBufferEntry speedsEntryInitial = new EKFSLAMBufferEntry(SimpleMatrix.identity(5), SimpleMatrix.identity(5), new ChassisSpeeds(), 0);
        EKFSLAMBufferEntry visionEntry = new EKFSLAMBufferEntry(SimpleMatrix.identity(5), SimpleMatrix.identity(5), new Transform3d(), 1, 1);
        EKFSLAMBufferEntry speedsEntryFinal = new EKFSLAMBufferEntry(SimpleMatrix.identity(5), SimpleMatrix.identity(5), new ChassisSpeeds(), 2);


        chunk.addEntry(speedsEntryInitial);
        chunk.addEntry(visionEntry);
        chunk.addEntry(speedsEntryFinal);

        assert(chunk.isComplete());
    }

    @Test
    public void testIncompleteChunk() {
        EKFSLAMBufferChunk chunk = new EKFSLAMBufferChunk();

        EKFSLAMBufferEntry speedsEntry = new EKFSLAMBufferEntry(SimpleMatrix.identity(5), SimpleMatrix.identity(5), new ChassisSpeeds(), 0);

        chunk.addEntry(speedsEntry);

        assert(!chunk.isComplete());

        EKFSLAMBufferEntry visionEntry = new EKFSLAMBufferEntry(SimpleMatrix.identity(5), SimpleMatrix.identity(5), new Transform3d(), 1, 1);

        chunk.addEntry(visionEntry);

        assert(!chunk.isComplete());
    }

    @Test
    public void testOrder() {
        EKFSLAMBufferChunk chunk = new EKFSLAMBufferChunk();

        EKFSLAMBufferEntry speedsEntryInitial = new EKFSLAMBufferEntry(SimpleMatrix.identity(5), SimpleMatrix.identity(5), new ChassisSpeeds(), 0);
        EKFSLAMBufferEntry visionEntry = new EKFSLAMBufferEntry(SimpleMatrix.identity(5), SimpleMatrix.identity(5), new Transform3d(), 1, 1);
        EKFSLAMBufferEntry speedsEntryFinal = new EKFSLAMBufferEntry(SimpleMatrix.identity(5), SimpleMatrix.identity(5), new ChassisSpeeds(), 2);

        chunk.addEntry(speedsEntryInitial);
        chunk.addEntry(speedsEntryFinal);

        assert(chunk.isComplete());

        chunk.addEntry(visionEntry);

        assert(chunk.chunk.get(0).isSpeedsEntry());
        assert(chunk.chunk.get(1).isVisionEntry());
        assert(chunk.chunk.get(2).isSpeedsEntry());
    }

    @Test
    public void testOldVisionEntry() {
        EKFSLAMBufferChunk chunk = new EKFSLAMBufferChunk();

        EKFSLAMBufferEntry speedsEntryInitial = new EKFSLAMBufferEntry(SimpleMatrix.identity(5), SimpleMatrix.identity(5), new ChassisSpeeds(), 1);
        EKFSLAMBufferEntry oldVisionEntry = new EKFSLAMBufferEntry(SimpleMatrix.identity(5), SimpleMatrix.identity(5), new Transform3d(), 1, 0);
        EKFSLAMBufferEntry speedsEntryFinal = new EKFSLAMBufferEntry(SimpleMatrix.identity(5), SimpleMatrix.identity(5), new ChassisSpeeds(), 2);

        chunk.addEntry(speedsEntryInitial);

        chunk.addEntry(oldVisionEntry);

        chunk.addEntry(speedsEntryFinal);

        assert(chunk.chunk.size() == 2);
    }

    @Test
    public void testOldSpeedsEntry() {
        EKFSLAMBufferChunk chunk = new EKFSLAMBufferChunk();

        EKFSLAMBufferEntry speedsEntryInitial = new EKFSLAMBufferEntry(SimpleMatrix.identity(5), SimpleMatrix.identity(5), new ChassisSpeeds(), 1);
        EKFSLAMBufferEntry oldSpeedsEntry = new EKFSLAMBufferEntry(SimpleMatrix.identity(5), SimpleMatrix.identity(5), new ChassisSpeeds(), 0);
        EKFSLAMBufferEntry speedsEntryFinal = new EKFSLAMBufferEntry(SimpleMatrix.identity(5), SimpleMatrix.identity(5), new ChassisSpeeds(), 2);

        chunk.addEntry(speedsEntryInitial);

        chunk.addEntry(oldSpeedsEntry);

        chunk.addEntry(speedsEntryFinal);

        assert(chunk.chunk.size() == 2);
    }

    @Test
    public void testIntermidiateSpeedsEntry() {
        EKFSLAMBufferChunk chunk = new EKFSLAMBufferChunk();

        EKFSLAMBufferEntry speedsEntryInitial = new EKFSLAMBufferEntry(SimpleMatrix.identity(5), SimpleMatrix.identity(5), new ChassisSpeeds(), 0);
        EKFSLAMBufferEntry speedsEntryIntermidiate = new EKFSLAMBufferEntry(SimpleMatrix.identity(5), SimpleMatrix.identity(5), new ChassisSpeeds(), 1);
        EKFSLAMBufferEntry speedsEntryFinal = new EKFSLAMBufferEntry(SimpleMatrix.identity(5), SimpleMatrix.identity(5), new ChassisSpeeds(), 2);

        chunk.addEntry(speedsEntryInitial);

        chunk.addEntry(speedsEntryIntermidiate);

        chunk.addEntry(speedsEntryFinal);

        assert(chunk.chunk.size() == 2);
    }
}
