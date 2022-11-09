#ifndef __PQ_H__
#define __PQ_H__

typedef void* (*Job)(void* params);

/**
 * The type of the nodes for this priority queue.
 * @param id The ID of the corresponding ticket-holder (in this case, a Publisher).
 * @param priority The value which will be used for sorting the priority queue.
 */
typedef struct Ticket {
    uint8_t  rate;
    uint32_t nextW;
    Job job;
} Ticket;


void insert(int rate, int nextW, Job job);
Ticket* peekMin();
Ticket* extractMin();

int getSize();

// FIXME: remove after test
//void printQueue();

#endif /* __PQ_H__ */
