#ifndef FILEUTIL_H
#define FILEUTIL_H

#include <stdio.h>

#include "DataNode.h"
#include "SPRING.h"

typedef struct customGestureItem{
    char *gestureName;
    int gestureFunction;
    struct customGestureItem * next;
}CustomGestureItem;

typedef struct custemGetsureList{
    CustomGestureItem* head;
    CustomGestureItem* tail;
    int length;
}CustomGestureList;

OriginalGesture *read_file_to_init_original_gesture(char * fileName);

CustomGestureParameter read_custom_gesture_parameter(char * fileName);

void write_queue_to_file(char * fileName, SqQueue * queue);

void write_list_to_file(char * fileName, DataHeadNode *pHead);

void write_pkt_to_file(char * fileName, PktData pktData);

void write_mag_to_file(char * fileName, double x[], double y[], double z[], double heading[], int len) ;

void save_user_template(char * fileName, DataHeadNode *dataHeadNode);

void save_user_template_parameter(double threshold, int timeSpan, char * name);

void insert_new_custom_gesture_item(CustomGestureItem item);

void load_custom_gesture_list(CustomGestureList *cList);

#endif // FILEUTIL_H
