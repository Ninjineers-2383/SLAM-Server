package com.team2383.SLAM.server.SLAM.buffer;

import java.util.ArrayList;

public class EKFSLAMBufferChunk {
        public ArrayList<EKFSLAMBufferEntry> chunk = new ArrayList<EKFSLAMBufferEntry>();

        public EKFSLAMBufferChunk() {
        }

        public void addEntry(EKFSLAMBufferEntry entry) {
            // If the entry is before the first entry in the chunk, throw it away, else if the chunk is empty, throw out vision measurments
            if (!chunk.isEmpty()) {
                if (entry.timestamp < chunk.get(0).timestamp) {
                    return;
                }
            } else {
                if (entry.isVisionEntry()) {
                    return;
                }
            }


            // If the chunk is incomplete, add the entry
            if (!isComplete()) {
                chunk.add(entry);
                return;
            }

            // Since we know the chunk is complete, check if the entry is a vision entry and if it is, check if it is before the final entry, 
            // if so, put it in the correct order in time, if it is after, it does not belong in this chunk, so throw it away
            if (entry.isVisionEntry()) {
                if (entry.timestamp < getFinalEntry().timestamp) {
                    chunk.add(chunk.size() - 1, entry);
                    return;
                }
            }

            // Since we know the entry is not a vision entry, if it is before the final entry in time, throw it away
            if (entry.timestamp < getFinalEntry().timestamp) {
                return;
            }
        }

        public EKFSLAMBufferEntry getFinalEntry() {
            return chunk.get(chunk.size() - 1);
        }

        public boolean isEmpty() {
            return chunk.isEmpty();
        }

        public boolean isComplete() {
            if (chunk.size() < 2) {
                return false;
            }
            if (chunk.get(0).isSpeedsEntry() && chunk.get(chunk.size() - 1).isSpeedsEntry()) {
                return true;
            } else {
                return false;
            }
        }
    }
