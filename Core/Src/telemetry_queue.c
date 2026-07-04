#include "telemetry_queue.h"

// The static buffer survives STOP mode
static TelemetryRecord_t buffer[QUEUE_MAX_SIZE];
static uint16_t head = 0; // Where we write new data
static uint16_t tail = 0; // Where we read old data
static uint16_t count = 0; // How many items are waiting

void Queue_Init(void) {
    head = 0;
    tail = 0;
    count = 0;
}

// Simple ringbuffer
bool Queue_Push(TelemetryRecord_t record) {
    if (count >= QUEUE_MAX_SIZE) {
        tail = (tail + 1) % QUEUE_MAX_SIZE;
        count--;
    }

    buffer[head] = record;
    head = (head + 1) % QUEUE_MAX_SIZE;
    count++;

    return true;
}

bool Queue_Peek(TelemetryRecord_t *record) {
    if (count == 0) return false;

    *record = buffer[tail];
    return true;
}

void Queue_Pop(void) {
    if (count > 0) {
        tail = (tail + 1) % QUEUE_MAX_SIZE;
        count--;
    }
}

bool Queue_HasPendingData(void) {
    return (count > 0);
}
