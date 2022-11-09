#include <stdlib.h>
#include <string.h>
#include "datastructures.h"
#include "pq.h"




/** The array backing this min-heap-based priority queue. */
Ticket pq[MAX_PUB];

/** The current size of the priority queue. */
int size = 0;

/** The current minimum (root) of the priority queue. */
Ticket min;

/**
 * Gets the index of the parent node of node i.
 * @param i The index of the child node.
 * @returns The index of the parent node.
 */
inline int parent(int i) {
    return i >> 1;
}

/**
 * Gets the index of the left child of node i.
 * @param i The index of the parent node.
 * @returns The index of the left child of node i.
 */
inline int left(int i) {
	return i << 1;
}

/**
 * Gets the index of the right child of node i.
 * @param i The index of the parent node.
 * @returns The index of the right child of node i.
 */
inline int right(int i) {
	return (i << 1) | 1;
}

/**
 * Enables the min-heap data structure by moving the nodes of the tree based on
 * their nextW in order to maintain the rules of a min-heap.
 * @param i The index of the node to heapify.
 */
void minHeapify(int i) {
	int l = left(i);
	int r = right(i);
	int smallest = i;
	if (l < size && pq[l].nextW < pq[i].nextW)
		smallest = l;
	if (r < size && pq[r].nextW < pq[smallest].nextW)
		smallest = r;

	if (smallest != i) {
		Ticket ticket;
		memcpy(&ticket, &pq[i], sizeof(Ticket));
		memcpy(&pq[i], &pq[smallest], sizeof(Ticket));
		memcpy(&pq[smallest], &ticket, sizeof(Ticket));
		minHeapify(smallest);
	}
}

/**
 * Removes the minimum (highest) nextW Ticket from the queue.
 * @returns A pointer to the minimum (highest) nextW Ticket.
 */
Ticket* extractMin() {
	if (size < 1) return NULL;
	
	memcpy(&min, &pq[0], sizeof(Ticket));
	size--;
	memcpy(&pq[0], &pq[size], sizeof(Ticket));
	minHeapify(0);
	return &min;
}

/**
 * Returns a pointer to the minimum (highest) nextW Ticket from the queue
 * without removing the Ticket from the queue.
 * @returns A pointer to the minimum (highest) nextW Ticket.
 */
Ticket* peekMin() {
	return &pq[0];
}

/**
 * Moves a Ticket through the heap until it is in its proper place.
 * @param i The index of the Ticket in the nextW queue.
 * @param nextW The nextW of the Ticket.
 */
 // FIXME: is this right?
void decreaseKey(int i, uint32_t nextW) {
	if (nextW > pq[i].nextW) return; // ERROR
	
	pq[i].nextW = nextW;
	while (i > 0 && pq[parent(i)].nextW > pq[i].nextW) {
		Ticket ticket;
		memcpy(&ticket, &pq[i], sizeof(Ticket));
		memcpy(&pq[i], &pq[parent(i)], sizeof(Ticket));
		memcpy(&pq[parent(i)], &ticket, sizeof(Ticket));
		i = parent(i);
	}
}

/**
 * Inserts a Ticket into the nextW queue. The structure is modified to
 * maintain a proper min-heap.
 * @param ref_id The id or index of the corresponding ticket-holder (in this case, 
 *           a Publisher).
 * @param nextW The nextW of the new Ticket.
 */
void insert(uint32_t rate, uint32_t nextW, Job job) {
	pq[size].rate = rate;
	pq[size].nextW = nextW;
	pq[size].job = job;
	size++;
	decreaseKey(size - 1, nextW);
}

int getSize() {
    return size;
}

/*
// FIXME: remove after test
void printQueue() {
    DEBUG_PRINT("< ");

    int i = 0;
    for (i = 0; i < PQ_CAPACITY; i++) {
        if (&pq[i] == NULL) DEBUG_PRINT("[NULL]");
        else DEBUG("[%d,%d]", pq[i].ref_id, pq[i].priority);
        if (i < PQ_CAPACITY-1) DEBUG_PRINT(", ");
    }

    DEBUG_PRINT(" >\n");
}*/
