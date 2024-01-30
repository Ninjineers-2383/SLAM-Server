package com.team2383.SLAM.server.common.buffer;

import java.util.TreeSet;
import java.util.Iterator;

public class TimeSyncedBuffer {
    private TreeSet<BufferEntry> buffer = new TreeSet<BufferEntry>();

    public void addEntry(BufferEntry entry) {
        buffer.add(entry);
    }

    public Iterator<BufferEntry> getIterator() {
        return buffer.iterator();
    }
}
