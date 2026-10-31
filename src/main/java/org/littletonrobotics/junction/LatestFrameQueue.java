package org.littletonrobotics.junction;

import java.util.concurrent.ArrayBlockingQueue;

/** Single-producer queue: a slow receiver loses old frames rather than blocking robot control. */
final class LatestFrameQueue extends ArrayBlockingQueue<LogTable> {
    static final int HARDWARE_CAPACITY = 8;
    private static final long serialVersionUID = 1L;
    private volatile long droppedFrames;
    private final int capacity;

    LatestFrameQueue(int capacity) {
        super(capacity);
        this.capacity = capacity;
    }

    /** Called only by Logger's robot-thread producer, never by the receiver. */
    void offerLatest(LogTable frame) {
        // The receiver can only remove entries, so after one eviction the retry has room.
        if (!offer(frame)) {
            if (poll() != null) droppedFrames++;
            if (!offer(frame)) throw new IllegalStateException("Multiple log producers are unsupported");
        }
    }

    long droppedFrames() { return droppedFrames; }
    int capacity() { return capacity; }
}
