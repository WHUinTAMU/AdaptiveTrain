#ifndef FILEUTIL_H
#define FILEUTIL_H

#include <stdio.h>

#include "DataNode.h"
#include "SPRING.h"

typedef struct customGestureItem{
    char *gestureName;
    int gestureFunction;
    int magNum;
    struct customGestureItem * next;
}CustomGestureItem;

typedef struct custemGetsureList{
    CustomGestureItem* head;
    CustomGestureItem* tail;
    int length;
}CustomGestureList;

OriginalGesture *read_file_to_init_original_gesture(char * fileName, bool isMag, bool isMagTemlpate, int magTemplateNum, char *gestureName);

CustomGestureParameter read_custom_gesture_parameter(char * fileName);

void write_queue_to_file(char * fileName, SqQueue * queue);

void write_list_to_file(char * fileName, DataHeadNode *pHead);

void write_pkt_to_file(char * fileName, PktData pktData);

void write_mag_to_file(char * fileName, double x[], double y[], double z[], double heading[], int len) ;

void save_user_template(char * fileName, DataHeadNode *dataHeadNode);

void save_user_template_parameter(double threshold, int timeSpan, char * name);

void insert_new_custom_gesture_item(CustomGestureItem item);

void load_custom_gesture_list(CustomGestureList *cList);

/**
*TASK: save target template into a file for later use
*filename: name of the file
*userMagData: the target template created by the user
*/
void save_mag_template(char * fileName, AverageList *userMagData);

void save_path_template(char *,WarpingPathTypeItem *);

#endif // FILEUTIL_H
