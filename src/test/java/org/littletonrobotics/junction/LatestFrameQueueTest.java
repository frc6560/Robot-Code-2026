package org.littletonrobotics.junction;

import static org.junit.jupiter.api.Assertions.*;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.Test;

class LatestFrameQueueTest {
    @Test void sustainedSaturationKeepsOnlyEightNewestFrames() {
        var queue = new LatestFrameQueue(LatestFrameQueue.HARDWARE_CAPACITY);
        for (int i = 0; i < 10000; i++) {
            queue.offerLatest(new LogTable(i));
            assertTrue(queue.size() <= 8);
        }
        assertEquals(8, queue.capacity());
        assertEquals(9992, queue.droppedFrames());
        for (int i = 9992; i < 10000; i++) assertEquals(i, queue.poll().getTimestamp());
        assertNull(queue.poll());
    }

    @Test void queuedSnapshotRemainsIndependentOfCurrentValues() {
        var queue = new LatestFrameQueue(LatestFrameQueue.HARDWARE_CAPACITY);
        var current = new LogTable(1);
        current.put("value", 12.0);
        queue.offerLatest(LogTable.clone(current));
        current.put("value", 99.0);
        current.setTimestamp(2);
        var snapshot = queue.poll();
        assertEquals(12.0, snapshot.get("value", 0.0));
        assertEquals(1, snapshot.getTimestamp());
    }

    @Test void actualReceiverStallCannotGrowBacklogAndRecoversWithLatestData() throws Exception {
        var queue = new LatestFrameQueue(LatestFrameQueue.HARDWARE_CAPACITY);
        var receiver = new ReceiverThread(queue);
        var entered = new CountDownLatch(1);
        var release = new CountDownLatch(1);
        var receivedNine = new CountDownLatch(9);
        List<Long> received = Collections.synchronizedList(new ArrayList<>());
        receiver.addDataReceiver(new LogDataReceiver() {
            @Override public void putTable(LogTable table) throws InterruptedException {
                if (table.getTimestamp() == 1) {
                    entered.countDown();
                    release.await();
                }
                received.add(table.getTimestamp());
                receivedNine.countDown();
            }
        });
        receiver.start();
        try {
            queue.offerLatest(new LogTable(1));
            assertTrue(entered.await(2, TimeUnit.SECONDS));
            for (int i = 2; i <= 1001; i++) queue.offerLatest(new LogTable(i));
            assertEquals(8, queue.size());
            assertEquals(992, queue.droppedFrames());
            release.countDown();
            assertTrue(receivedNine.await(2, TimeUnit.SECONDS));
            assertEquals(List.of(1L, 994L, 995L, 996L, 997L, 998L, 999L, 1000L, 1001L), received);
            assertEquals(0, queue.size());
        } finally {
            release.countDown();
            receiver.interrupt();
            receiver.join(2000);
        }
        assertFalse(receiver.isAlive());
    }

    @Test void desktopLoggerUsesCompatibleCapacityAndPatchedApi() {
        assertEquals(500, Logger.getReceiverQueueCapacity());
        assertEquals(0, Logger.getDroppedFrameCount());
    }
}
