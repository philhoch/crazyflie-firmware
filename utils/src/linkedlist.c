#include "../interface/linkedlist.h"
#include "FreeRTOS.h"

linkedlistItem* getLinkedlistFrontItem(linkedlist* list) {
    return list->front;
}

linkedlistItem* getLinkedlistRearItem(linkedlist* list) {
    return list->rear;
}

void linkedlistAddItem(linkedlist* list, linkedlistItem* item) {
    if(list->rear == 0) {
        list->front = item;
        list->rear = item;
    } else {
        list->rear->next = item;
        list->rear = item;
    }
    list->elements++;
}

void* getLinkedlistItemValue(linkedlistItem* item) {
    return item->item;
}

linkedlistItem* getLinkedlistItemNext(linkedlistItem* item) {
    return item->next;
}

uint32_t getLinkedListSize(linkedlist* list) {
    return list->elements;
}

void clearLinkedList(linkedlist* list) {
    uint32_t i = 0;
    for(i = 0; i < list->elements; ++i) {
        linkedlistItem* item = list->front;
        list->front = item->next;
        vPortFree(item->item);
        vPortFree(item);
    }
    list->front = 0;
    list->rear = 0;
    list->elements = 0;
}

linkedlistItem* linkedListPopFront(linkedlist *list) {
    if(list->front != 0) {
        linkedlistItem* front = list->front;
        if(front->next != 0) {
            list->front = front->next;
        } else {
            list->front = 0;
        }
        list->elements--;
        return front;
    }
    return 0;
}

void deleteLinkedlistItem(linkedlistItem *item) {
    vPortFree(item->item);
    vPortFree(item);
}
