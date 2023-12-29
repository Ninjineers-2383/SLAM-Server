package com.team2383.SLAM.server.SLAM.buffer;

import java.util.ArrayList;
import java.util.Optional;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class EKFSLAMBuffer {
    private ArrayList<EKFSLAMBufferEntry> buffer = new ArrayList<EKFSLAMBufferEntry>();
    private int speedsCounter;

    public EKFSLAMBuffer() {
    }

    public void addVisionEntry(EKFSLAMBufferEntry entry) {
        if (isEmpty()) {
            return;
        }

        if (entry.timestamp < buffer.get(0).timestamp) {
            return;
        }

        if (entry.timestamp > buffer.get(buffer.size() - 1).timestamp) {
            buffer.add(entry);
            return;
        }

        for (int i = 0; i < buffer.size() - 1; i++) {
            if (entry.timestamp > buffer.get(i).timestamp && entry.timestamp < buffer.get(i + 1).timestamp) {
                buffer.add(i + 1, entry);
                return;
            }
        }
    }

    public void addSpeedsEntry(EKFSLAMBufferEntry entry) {
        if (isEmpty()) {
            buffer.add(entry);
            speedsCounter++;
            return;
        }

        if (entry.timestamp < buffer.get(0).timestamp) {
            return;
        }

        if (speedsCounter == 2) {
            if (buffer.get(getSecondSpeedsIndex()).timestamp > entry.timestamp) {
                return;
            }
              

            refreshBuffer();
            speedsCounter = 1;
            buffer.add(entry);
            return;
        }

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

    public void refreshBuffer() {
        int secondSpeedsIndex = getSecondSpeedsIndex();

        // Delete every entry until the next speeds entry
        for (int i = 0; i < secondSpeedsIndex; i++) {
            buffer.remove(i);
        }
       
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

    public Optional<SimpleMatrix> getLatestMu() {
        for (EKFSLAMBufferEntry entry : buffer) {
            if (entry.mu.isPresent()) {
                return entry.mu;
            }
        }

        return Optional.empty();
    }

    public Optional<SimpleMatrix> getLatestSigma() {
        for (EKFSLAMBufferEntry entry : buffer) {
            if (entry.sigma.isPresent()) {
                return entry.sigma;
            }
        }

        return Optional.empty();
    }


    public ChassisSpeeds interpolateSpeeds(int index) {
        if (buffer.isEmpty() || !hasSecondSpeedsEntry()) {
            return new ChassisSpeeds();
        }

        int secondSpeedsIndex = getSecondSpeedsIndex();

        ChassisSpeeds speedsInitial = buffer.get(0).speeds.get();
        ChassisSpeeds speedsFinal = buffer.get(secondSpeedsIndex).speeds.get();

        double timeFinal = get(secondSpeedsIndex).timestamp;
        double timeInitial = get(0).timestamp;
        double timeIndex = get(index).timestamp;

        ChassisSpeeds interpolatedSpeeds = speedsFinal.minus(speedsInitial).div((timeFinal - timeInitial) * 0.5).times(((timeFinal - timeIndex) * 0.5) + timeIndex - timeInitial).plus(buffer.get(0).speeds.get());

        return interpolatedSpeeds;
    }
}
