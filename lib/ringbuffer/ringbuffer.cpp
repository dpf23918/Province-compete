// ==== ringbuffer.cpp ====
#include "ringbuffer.h"

void ringbuffer_init(ringbuffer_t* fifo, uint8_t* buffer, uint16_t size) {
    fifo->buffer = buffer;
    fifo->size = size;
    fifo->in = 0;
    fifo->out = 0;
}

uint16_t ringbuffer_getUsedSize(ringbuffer_t* fifo) {
    return (fifo->in >= fifo->out) ? 
        (fifo->in - fifo->out) : 
        (fifo->size - fifo->out + fifo->in);
}

uint16_t ringbuffer_getRemainSize(ringbuffer_t* fifo) {
    return fifo->size - ringbuffer_getUsedSize(fifo) - 1;
}

uint8_t ringbuffer_isEmpty(ringbuffer_t* fifo) {
    return (fifo->in == fifo->out);
}

void ringbuffer_in(ringbuffer_t* fifo, uint8_t* data, uint16_t len) {
    for (int i=0; i<len; i++) {
        fifo->buffer[fifo->in] = data[i];
        fifo->in = (fifo->in + 1) % fifo->size;
    }
}

uint8_t ringbuffer_in_check(ringbuffer_t* fifo, uint8_t* data, uint16_t len) {
    if (ringbuffer_getRemainSize(fifo) < len) return 1;
    ringbuffer_in(fifo, data, len);
    return 0;
}

uint16_t ringbuffer_out(ringbuffer_t* fifo, uint8_t* buf, uint16_t len) {
    uint16_t remain = ringbuffer_getUsedSize(fifo);
    if (remain > len) remain = len;
    
    for (int i=0; i<remain; i++) {
        buf[i] = fifo->buffer[fifo->out];
        fifo->out = (fifo->out + 1) % fifo->size;
    }
    return remain;
}