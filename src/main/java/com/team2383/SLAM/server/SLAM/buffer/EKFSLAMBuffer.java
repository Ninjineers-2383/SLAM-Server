package com.team2383.SLAM.server.SLAM.buffer;

import java.util.ArrayList;
import java.util.function.Function;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class EKFSLAMBuffer {
    private ArrayList<EKFSLAMBufferEntry> buffer = new ArrayList<EKFSLAMBufferEntry>();
    private int speedsCounter;

    private final Function<EKFSLAMBufferEntry, EKFSLAMState> getInitialSLAMState;

    public EKFSLAMBuffer(Function<EKFSLAMBufferEntry, EKFSLAMState> getInitialSLAMState) {
        this.getInitialSLAMState = getInitialSLAMState;
    }

    public void addVisionEntry(EKFSLAMBufferEntry entry) {
        // If the buffer is entirely empty and we get a vision measurement we need to
        // initialize the SLAM system with the first measurements
        if (isEmpty()) {
            EKFSLAMBufferEntry initialSpeedsEntry = new EKFSLAMBufferEntry(new ChassisSpeeds(), entry.timestamp);
            initialSpeedsEntry.updateMuAndSigma(getInitialSLAMState.apply(entry));
            buffer.add(initialSpeedsEntry);
            buffer.add(entry);
            speedsCounter = 1;

            return;
        }

        // If the vision measurement is outside the range of the buffer, ignore it
        if (entry.timestamp < buffer.get(0).timestamp) {
            return;
        }

        // If the vision measurement is newer than the newest entry, add it to the end
        if (entry.timestamp > buffer.get(buffer.size() - 1).timestamp) {
            buffer.add(entry);
            return;
        }

        // If the vision measurement is between two entries, add it between them
        for (int i = 0; i < buffer.size() - 1; i++) {
            if (entry.timestamp > buffer.get(i).timestamp && entry.timestamp < buffer.get(i + 1).timestamp) {
                buffer.add(i + 1, entry);
                return;
            }
        }
    }

    public void addSpeedsEntry(EKFSLAMBufferEntry entry) {
        // If the buffer is empty we want to ignore the speeds entry because we can't
        // start SLAM until we get a vision measurement
        if (isEmpty()) {
            return;
        }

        // If the speeds entry is older than the oldest entry in the buffer, ignore it
        if (entry.timestamp < buffer.get(0).timestamp) {
            return;
        }

        // If we currently have two speeds in the buffer (one full chunk)
        if (speedsCounter == 2) {
            // If this speeds entry is older than the newest speeds entry something went
            // horrible wrong on the network side and we need to ignore it
            if (buffer.get(getSecondSpeedsIndex()).timestamp > entry.timestamp) {
                return;
            }

            if (buffer.get(buffer.size() - 1).timestamp < entry.timestamp) {
                buffer.add(entry);
                speedsCounter++;
                clearBuffer();
                return;
            }

            // If this speeds entry is between two entries, add it between them
            for (int i = 0; i < buffer.size() - 1; i++) {
                if (entry.timestamp > buffer.get(i).timestamp && entry.timestamp < buffer.get(i + 1).timestamp) {
                    buffer.add(i + 1, entry);
                    speedsCounter++;
                    clearBuffer();
                    return;
                }
            }
            return;
        }

        // If we currently have one speeds in the buffer (half a chunk)
        if (speedsCounter == 1) {
            for (int i = 0; i < buffer.size() - 1; i++) {
                if (entry.timestamp > buffer.get(i).timestamp && entry.timestamp < buffer.get(i + 1).timestamp) {
                    buffer.add(i + 1, entry);
                    speedsCounter++;
                    return;
                }
            }
        }

        buffer.add(entry);
        speedsCounter++;
    }

    public void clearBuffer() {
        int secondSpeedsIndex = getSecondSpeedsIndex();

        // Delete every entry until the next speeds entry
        for (int i = 0; i < secondSpeedsIndex; i++) {
            buffer.remove(0);
        }
        speedsCounter--;
    }

    public int size() {
        return buffer.size();
    }

    public EKFSLAMBufferEntry get(int index) {
        return buffer.get(index);
    }

    public boolean isEmpty() {
        return buffer.isEmpty();
    }

    public boolean hasSecondSpeedsEntry() {
        return speedsCounter == 2;
    }

    public void empty() {
        buffer.clear();
    }

    public int getSecondSpeedsIndex() {
        for (int i = 1; i < buffer.size(); i++) {
            if (buffer.get(i).isSpeedsEntry()) {
                return i;
            }
        }

        return -1;
    }

    public int getEarliestEmptyState() {
        for (int i = 0; i < buffer.size(); i++) {
            if (!buffer.get(i).state.isPresent()) {
                return i;
            }
        }

        return buffer.size();
    }

    public ChassisSpeeds interpolateSpeeds(int index) {
        if (buffer.isEmpty() || !hasSecondSpeedsEntry()) {
            return new ChassisSpeeds();
        }

        int secondSpeedsIndex = getSecondSpeedsIndex();

        ChassisSpeeds speedsInitial = buffer.get(0).speeds.get().speeds();
        ChassisSpeeds speedsFinal = buffer.get(secondSpeedsIndex).speeds.get().speeds();

        double timeFinal = get(secondSpeedsIndex).timestamp;
        double timeInitial = get(0).timestamp;
        double timeIndex = get(index).timestamp;

        ChassisSpeeds interpolatedSpeeds = speedsFinal.minus(speedsInitial).div((timeFinal - timeInitial) * 0.5)
                .times(((timeFinal - timeIndex) * 0.5) + timeIndex - timeInitial)
                .plus(buffer.get(0).speeds.get().speeds());

        return interpolatedSpeeds;
    }
}
