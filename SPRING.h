#ifndef SPRING_H
#define SPRING_H

#include<stdio.h>
#include<stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <float.h>
#include "DataNode.h"
#include "LampController.h"

#define CLICK_THRESHOLD 800
#define CLICK_TIMELIMIT 300
#define CLICK_TYPE 10

#define TARGET_THRESHOLD 35
#define POINT_THRESHOLD 120
#define ROTATE_RIGHT_THRESHOLD 30
#define ROTATE_RIGHT_HALF_THRESHOLD 30
#define ROTATE_RIGHT_FULL_THRESHOLD 40
#define ROTATE_LEFT_HALF_THRESHOLD 25
#define ROTATE_LEFT_FULL_THRESHOLD 25
#define ROTATE_LEFT_THRESHOLD 55
#define SLIDE_OVER_THRESHOLD 30
#define STAND_UP_THRESHOLD 28
#define SIT_DOWN_THRESHOLD 20
#define WALK_THRESHOLD 50

#define TARGET_TIMELIMIT 0
#define POINT_TIMELIMIT 500
#define ROTATE_RIGHT_TIMELIMIT 400
#define ROTATE_RIGHT_HALF_TIMELIMIT 250
#define ROTATE_RIGHT_FULL_TIMELIMIT 400
#define ROTATE_LEFT_TIMELIMIT 400
#define ROTATE_LEFT_HALF_TIMELIMIT 250
#define ROTATE_LEFT_FULL_TIMELIMIT 400
#define SLIDE_OVER_TIMELIMIT 500
#define STAND_UP_TIMELIMIT 550
#define SIT_DOWN_TIMELIMIT 550
#define WALK_TIMELIMIT 800

#define NONE_TYPE -1
#define TARGET_TYPE 0
#define POINT_TYPE 1
#define ROTATE_RIGHT_HALF_TYPE 2
#define ROTATE_RIGHT_FULL_TYPE 3
#define ROTATE_LEFT_HALF_TYPE 4
#define ROTATE_LEFT_FULL_TYPE 5
#define SLIDE_OVER_TYPE 6
#define STAND_UP_TYPE 7
#define SIT_DOWN_TYPE 8
#define WALK_TYPE 9
#define CUSTOM_TYPE 11

#define DTW_NUM 10

#define PATH_LEFT 0
#define PATH_DOWN 1
#define PATH_CORNER 2

typedef struct warpingPathTypeItem{
    int type;
    struct warpingPathTypeItem *next;
    struct warpingPathTypeItem *pre;
    int position;
    int y;
}WarpingPathTypeItem;

typedef struct warpingPathItem{
    int x;
    int y;
    int fx;
    int fy;
    int path;
}WarpingPathItem;

typedef struct warpingPathList{
    int lengthY;
    WarpingPathItem *itemArray;
    int position;
    struct warpingPathList *next;
    struct warpingPathList *pre;
}WarpingPathList;

typedef struct warpingPathListHead{
    WarpingPathList *head;
    WarpingPathList *tail;
    int length;
    long int headNum;

}WarpingPathListHead;

/**
*original gesture model struct
*/
typedef struct originalGesture{
    DataHeadNode *head; //the head node of the list of the model data
    DataHeadNode *magListHead;
    int magNum;
    int m;// the length of the list
}OriginalGesture;

typedef struct customGestureParameter{
    double threshold;
    int timeSpan;
}CustomGestureParameter;

/**
*the process struct of one kind of gesture
*/
typedef struct gestureRecognitionProcess{
    OriginalGesture originalGesture;// the gesture model struct
    double *distanceArray;//array d
    double *distanceArrayLast;//array d'
    int *startArray;//array s
    int *startArrayLast;//array s'
    int *timeArray;//array s
    int *timeArrayLast;//array s'
    double dmin;// dmin
    int ts;//ts
    int te;//te
    long int times;//time stamp of the start
    long int timee;//time stamp of the end
    double threshold;// threshold
    int type;// determine which gesture it is
    long int timeLimit;// time limit of a piece of right data
    struct gestureRecognitionProcess *next;
    char *name;
    int functionNum;
    WarpingPathListHead warpingPathMetrixHead;
    WarpingPathItem *warpingPathArray;

}GRProcess;

/**
*the struct for averaging and averaged mag data.
*/
typedef struct averageList
{
 DataNode *head;
 struct averageList *next;
 struct averageList *pre;
 }AverageList;

/**
*the struct of the node of the stack for normalizing raw mag data.
*/
typedef struct Node
{
  DataNode *data;
  struct Node *pNext;
}NODE,*PNODE;

/**
*the struct of the stack for normalizing raw mag data.
*/
typedef struct Stack
{
    PNODE pTop;
    PNODE pBottom;
}STACK,*PSTACK;

/**
*TASK:the process struct of one kind of gesture
*grp:the process struct of the specific type of gesture
*xt:the current data inputed
*/
void update_array(GRProcess *grProcess, PktData xt, int position, bool usePath);

/**
*TASK:the main part of the DTW algorithm
*grProcess:the process struct of the specific type of gesture
*xt:the current data inputed
return: the front of the queue
*/
int SPRING(PktData xt, GRProcess *grProcess, int position, SqQueue *queue, bool isSkip, bool isWriteDistance, bool isPrint, bool usePath, WarpingPathTypeItem **pathList);

double getDegreeFromGyro(int start, int end, SqQueue *queue);

double compute_traditional_DTW(OriginalGesture *og, DataHeadNode *head);

/**
*TASK: normalize the user's mag data according to the warping path extracted from accelerometer and gyroscope data.
*wpTypeItemTail: item of the warping path
*queue: the raw mag data that needs to be normalized
*s: start of the part to be normalized in the queue
*e: end of the part to be normalized in the queue
*/
AverageList *Normalization(WarpingPathTypeItem *wpTypeItemTail, SqQueue *queue, int s, int e);

/**
*TASK:compute the distance between normalized user mag data and the target template
*normalizedUserData: the normalized mag data
*dhn: DataHeadNode for the target template
*/
double compute_magdata_distance(AverageList *normalizedUserData, DataHeadNode *dhn);

/**
*TASK: transfer raw user mag data in SqQueue into DataHeadNode, convenient for the normalization of user mag data
*queue: the raw mag data that needs to be transformed into DataHeadNode
*s: start of the part in the queue to be transformed
*e: end of the part in the queue to be transformed
*/
DataHeadNode *transferSqQueueToDhn(SqQueue *queue, int start, int end);

bool empty(PSTACK pS);

void push(PSTACK pS, DataNode *val);

bool pop(PSTACK pS, DataNode **pVal);


#endif // SPRING_H
