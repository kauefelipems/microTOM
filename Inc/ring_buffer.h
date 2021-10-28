/*
 * ring_buffer.h
 *
 *  Created on: 19 de out de 2021
 *      Author: kauefelipems
 */

#ifndef INC_RING_BUFFER_H_
#define INC_RING_BUFFER_H_
#include <stdio.h>

typedef enum {false, true} boolean;

// Opaque ring buffer structure
typedef struct ring_buf_t {
	uint16_t * buffer;
	size_t head;
	size_t tail;
	size_t max; //of the buffer
	boolean full;
} ring_buf_t;

typedef ring_buf_t* ring_handler_t;

/// Pass in a storage buffer and size
/// Returns a ring buffer handle
void ring_buf_init(ring_handler_t rbuf, uint16_t* buffer, size_t size);

/// Free a ring buffer structure.
/// Does not free data buffer; owner is responsible for that
void ring_buf_free(ring_handler_t buff_pointer);

/// Reset the ring buffer to empty, head == tail
void ring_buf_reset(ring_handler_t buff_pointer);

/// Put version 1 continues to add data if the buffer is full
/// Old data is overwritten
void ring_buf_put(ring_handler_t rbuf, uint16_t* data, size_t data_size);

/// Retrieve a value from the buffer
/// Returns 0 on success, -1 if the buffer is empty
int ring_buf_get(ring_handler_t rbuf, uint16_t* data, size_t data_size);

/// Returns true if the buffer is empty
boolean ring_buf_empty(ring_handler_t buff_pointer);

/// Returns true if the buffer is full
boolean ring_buf_full(ring_handler_t buff_pointer);

/// Returns the maximum capacity of the buffer
size_t ring_buf_capacity(ring_handler_t buff_pointer);

/// Returns the current number of elements in the buffer
size_t ring_buf_size(ring_handler_t buff_pointer);


#endif /* INC_RING_BUFFER_H_ */
