package team2383.SLAM;

import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Test;

import com.team2383.SLAM.server.SLAM.buffer.EKFSLAMBuffer;
import com.team2383.SLAM.server.SLAM.buffer.EKFSLAMBufferEntry;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class EKFSLAMBufferTest {

    @Test
    public void testVisionEntryOnCompleteChunk() {
        EKFSLAMBuffer buffer = new EKFSLAMBuffer();

        EKFSLAMBufferEntry speedsEntryInitial = new EKFSLAMBufferEntry(SimpleMatrix.identity(5), SimpleMatrix.identity(5), new ChassisSpeeds(), 0);
        EKFSLAMBufferEntry visionEntry = new EKFSLAMBufferEntry(SimpleMatrix.identity(5), SimpleMatrix.identity(5), new Transform3d(), 1, 1);
        EKFSLAMBufferEntry speedsEntryFinal = new EKFSLAMBufferEntry(SimpleMatrix.identity(5), SimpleMatrix.identity(5), new ChassisSpeeds(), 2);

        buffer.addEntry(speedsEntryInitial);
        buffer.addEntry(speedsEntryFinal);
        buffer.addEntry(visionEntry);

        assert(buffer.chunkArray[1].isEmpty());
        assert(buffer.chunkArray[0].chunk.get(2).isVisionEntry());
    }
}
