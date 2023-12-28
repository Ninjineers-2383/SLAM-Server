package com.team2383.SLAM.server.SLAM.buffer;

public class EKFSLAMBuffer {
    public EKFSLAMBufferChunk[] chunkArray = new EKFSLAMBufferChunk[2];

    public EKFSLAMBuffer() {
        chunkArray[0] = new EKFSLAMBufferChunk();
        chunkArray[1] = new EKFSLAMBufferChunk();
    }

    public void addEntry(EKFSLAMBufferEntry entry) {
        // Check if the first chunk is complete, if it isn't, add the entry to it and return
        if (!chunkArray[0].isComplete()) {
            chunkArray[0].addEntry(entry);
            return;
        }

        // Check if the entry is a vision entry and if it is, check if it's timestamp is before the first chunk's final entry, if so, 
        //   add it to the first chunk and return, otherwise, add it to the second chunk and return
        if (entry.isVisionEntry()) {
            if (entry.timestamp < chunkArray[0].getFinalEntry().timestamp) {
                chunkArray[0].addEntry(entry);
                return;
            } else {
                chunkArray[1].addEntry(entry);
                return;
            } 
        }

        if (entry.timestamp < chunkArray[0].getFinalEntry().timestamp) {
            if (entry.isSpeedsEntry()) {
                return;
            } else {
                chunkArray[0].addEntry(entry);
                return;
            }
        } else {
            if (entry.isSpeedsEntry()) {
                chunkArray[1].addEntry(entry);
                return;
            } else {
                return;
            }
            chunkArray[1].addEntry(entry);
            return;
        }
    }
}
