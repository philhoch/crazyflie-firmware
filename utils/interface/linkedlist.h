#ifndef LINKEDLIST_H
#define LINKEDLIST_H

struct LINKEDLIST_ITEM {
    //float itemValue;
    void* item;
    struct LINKEDLIST_ITEM *next;
};

typedef struct LINKEDLIST_ITEM linkedlistItem;

struct LINKEDLIST {
    uint32_t elements;
    struct LINKEDLIST_ITEM *front;
    struct LINKEDLIST_ITEM *rear;
};

typedef struct LINKEDLIST linkedlist;

linkedlistItem* getLinkedlistFrontItem(linkedlist* list);
linkedlistItem* getLinkedlistRearItem(linkedlist* list);
void linkedlistAddItem(linkedlist* list, linkedlistItem* item);
void *getLinkedlistItemValue(linkedlistItem* item);
linkedlistItem* getLinkedlistItemNext(linkedlistItem* item);
uint32_t getLinkedListSize(linkedlist* list);
void clearLinkedList(linkedlist* list);
linkedlistItem* linkedListPopFront(linkedlist *list);
void deleteLinkedlistItem(linkedlistItem *item);




#endif // LINKEDLIST_H
