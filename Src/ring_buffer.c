/*
 * ring_buffer->c
 *
 *  Created on: 19 de out de 2021
 *      Author: kauefelipems
 */


// The hidden definition of the ring buffer structure

#include "ring_buffer.h"

static void advance_pointer(ring_handler_t rbuf);
static void retreat_pointer(ring_handler_t rbuf);

void ring_buf_init(ring_handler_t rbuf, uint16_t* buffer, size_t size)
{
	rbuf->max = size;
	rbuf->buffer = buffer;
	ring_buf_reset(rbuf);
}

void ring_buf_reset(ring_handler_t rbuf)
{

    rbuf->head = 0;
    rbuf->tail = 0;
    rbuf->full = false;
}

boolean ring_buf_full(ring_handler_t rbuf)
{

    return rbuf->full;
}

boolean ring_buf_empty(ring_handler_t rbuf)
{
	if ((rbuf->full == false) && (rbuf->head == rbuf->tail))
		return true;
	else
		return false;
}

size_t ring_buf_capacity(ring_handler_t rbuf)
{

	return rbuf->max;
}

size_t ring_buf_size(ring_handler_t rbuf)
{

	size_t size = rbuf->max;

	if((rbuf->full == false))
	{
		if(rbuf->head >= rbuf->tail)
		{
			size = (rbuf->head - rbuf->tail);
		}
		else
		{
			size = (rbuf->max + rbuf->head - rbuf->tail);
		}
	}

	return size;
}

void ring_buf_put(ring_handler_t rbuf, uint16_t* data, size_t data_size)
{
	for (size_t i = 0; i < data_size; i++){
		rbuf->buffer[rbuf->head] = data[i];
		advance_pointer(rbuf);
	}
}

int ring_buf_get(ring_handler_t rbuf, uint16_t* data, size_t data_size)
{
	int r = -1;

	for (size_t i = 0; i <= data_size; i++){
		if(ring_buf_empty(rbuf) == false){

			*data = rbuf->buffer[rbuf->tail];
			retreat_pointer(rbuf);

			r = 0;
		}
	}

    return r;
}

//Helper Functions
static void advance_pointer(ring_handler_t rbuf)
{

	if(rbuf->full == true)
   	{
		if (++(rbuf->tail) == rbuf->max) rbuf->tail = 0;
	}

	if (++(rbuf->head) == rbuf->max) rbuf->head = 0;

	rbuf->full = (rbuf->head == rbuf->tail);
}

static void retreat_pointer(ring_handler_t rbuf)
{
	rbuf->full = false;
	if (++(rbuf->tail) == rbuf->max)
	{
		rbuf->tail = 0;
	}
}


