#include "FreeRTOS.h"
#include "ll.h"

#include "portable.h"

Item* addItem(Item* prev, int data) {
    Item* item = (Item*)pvPortMalloc(sizeof(Item));
    item->data = data;
    item->next = prev;
    return item;
}

Item* removeItem(Item* item) {
    if (item == NULL) return NULL;
    Item* head = item->next;
    vPortFree(item);
    return head;
}

/*
void printList(Item* item) {
    while (item != NULL) {
        DEBUG_PRINT("[%d]", item->data);
        if (item->next != NULL) DEBUG_PRINT(" -> ");
        item = item->next;
    }
    DEBUG_PRINT("\n");
}
 */

