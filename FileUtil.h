#ifndef FILEUTIL_H
#define FILEUTIL_H

#include <stdio.h>

#include "DataNode.h"
#include "SPRING.h"

void write_queue_to_file(char * fileName, SqQueue * queue);

void write_list_to_file(char * fileName, DataHeadNode *pHead);

void write_pkt_to_file(char * fileName, PktData pktData);

void write_mag_to_file(char * fileName, double x[], double y[], double z[], double heading[], int len) ;

/**
*the structure for a custom gesture
*/
typedef struct customGestureItem
{
    char *gestureName;                  //gesture name
    int gestureFunction;                //the num of the function ON_OFF_TYPE BRI_UP_TYPE BRI_DOWN_TYPE HUE_UP_TYPE HUE_DOWN_TYPE
    int magNum;                         //the number of the mag template of this gesture
    struct customGestureItem * next;
} CustomGestureItem;

/**
*the structure for the custom gesture list
*/
typedef struct customGetsureList
{
    CustomGestureItem* head;
    CustomGestureItem* tail;
    int length;
} CustomGestureList;


/**
*TASK:load a gesture from txt file
*fileName
*isMag:whether this txt file contain mag data
*isMagTemlpate:whether need to load mag template file
*magTemplateNum:number of mag template
*gestureName: gesture name, in order to load mag template
*
*/
OriginalGesture *read_file_to_init_original_gesture(char * fileName, bool isMag, bool isMagTemlpate, int magTemplateNum, char *gestureName);

/**
*TASK:load the parameter of a gesture(threshold and time limit)
*/
CustomGestureParameter read_custom_gesture_parameter(char * fileName);

/**
*save a new gesture into list.txt
*/
void insert_new_custom_gesture_item(CustomGestureItem item);

/**
*load all gesture from list.txt
*/
void load_custom_gesture_list(CustomGestureList *cList);

/**
*save user acc and gyro and mag template into txt file
*/
void save_user_template(char * fileName, DataHeadNode *dataHeadNode);

/**
*save user gesture parameters into txt file
*/
void save_user_template_parameter(double threshold, int timeSpan, char * name);

/**
*TASK: save target template into a file for later use
*filename: name of the file
*userMagData: the target template created by the user
*/
void save_mag_template(char * fileName, AverageList *userMagData);

/**
*save a path just for debug
*/
void save_path_template(char *,WarpingPathTypeItem *);

#endif // FILEUTIL_H
