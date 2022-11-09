#ifndef __LL_H__
#define __LL_H__


/**
 * The type of the nodes for this linked list.
 * @param data The data held by the item (to be replaced with a Subscriber).
 * @param next A pointer to the next item in the linked list.
 */
typedef struct Item {
    int data;
    struct Item* next;
} Item;

Item* addItem(Item* prev, int data);
Item* removeItem(Item* item);

//void printList(Item* item);

#endif /* __LL_H__ */
