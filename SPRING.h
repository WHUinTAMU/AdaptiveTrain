#ifndef SPRING_H
#define SPRING_H

#include<stdio.h>
#include<stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <float.h>
#include "DataNode.h"
#include "LampController.h"

//parameter of click action
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

//define for warping path type
#define PATH_LEFT 0
#define PATH_DOWN 1
#define PATH_CORNER 2

/**
*the structure for the item in the list of warping path
*/
typedef struct warpingPathTypeItem{
    int type;                           //PATH_LEFT, PATH_DOWN or PATH_CORNER
    struct warpingPathTypeItem *next;   //point for making list
    struct warpingPathTypeItem *pre;
    int position;                       // the position of x
    int y;                              // the position of y
}WarpingPathTypeItem;

/**
*the structure for the item in the matrix of dynamic time warping
*/
typedef struct warpingPathItem{
    int x;                              //the position of x
    int y;                              //the position of y
    int fx;                             //the position of x of the last place (where the current data is from)
    int fy;                             //the position of y of the last place (where the current data is from)
    int path;                           //PATH_LEFT, PATH_DOWN or PATH_CORNER
}WarpingPathItem;

/**
*the structure of a single vertical list in the dynamic time warping matrix
*/
typedef struct warpingPathList{
    int lengthY;                        //the length of this list in the y direction
    WarpingPathItem *itemArray;         //the head point of the list
    int position;                       //the x position
    struct warpingPathList *next;       //point for making list
    struct warpingPathList *pre;
}WarpingPathList;

/**
*the structure of the dynamic time warping matrix
*/
typedef struct warpingPathListHead{
    WarpingPathList *head;
    WarpingPathList *tail;
    int length;                         //the length of the matrix in x direction
    long int headNum;                   //the position of the first data in the matrix in the user data queue

}WarpingPathListHead;

/**
*original gesture model struct
*/
typedef struct originalGesture{
    DataHeadNode *head;                 //the head node of the list of the model data
    DataHeadNode *magListHead;
    int magNum;
    int m;                              // the length of the list
}OriginalGesture;

/**
*the structure for custom gesture
*/
typedef struct customGestureParameter{
    double threshold;
    int timeSpan;
}CustomGestureParameter;

/**
*the process struct of one kind of gesture
*/
typedef struct gestureRecognitionProcess{
    OriginalGesture originalGesture;    // the gesture model struct
    double *distanceArray;              //array d
    double *distanceArrayLast;          //array d'
    int *startArray;                    //array s
    int *startArrayLast;                //array s'
    int *timeArray;                     //array s
    int *timeArrayLast;                 //array s'
    double dmin;                        // dmin
    int ts;                             //ts
    int te;                             //te
    long int times;                     //time stamp of the start
    long int timee;                     //time stamp of the end
    double threshold;                   // threshold
    int type;                           // determine which gesture it is
    long int timeLimit;                 // time limit of a piece of right data
    struct gestureRecognitionProcess *next;
    char *name;                         //the name of the gesture
    int functionNum;                    //the num of the function ON_OFF_TYPE BRI_UP_TYPE BRI_DOWN_TYPE HUE_UP_TYPE HUE_DOWN_TYPE
    WarpingPathListHead warpingPathMetrixHead;//the dynamic time warping matrix for the temporary optimal subsequence
    WarpingPathItem *warpingPathArray;  //the current path list

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
*position:the position of thr current data in the queue
*usePath:whether need to compute the warping path
*/
void update_array(GRProcess *grProcess, PktData xt, int position, bool usePath);

/**
*TASK:the main part of the SPRING DTW algorithm
*grProcess:the process struct of the specific type of gesture
*xt:the current data
*position:the position of the current data in the queue
*isSkip:(ignore this variable)
*isWriteDistance:whether output the DTW distance to a txt file
*isPrint:whether print the DTW details in the screen
*usePath:whether compute the warping path
*pathList:the point of the variable to contain the warping path
*/
int SPRING(PktData xt, GRProcess *grProcess, int position, SqQueue *queue, bool isSkip, bool isWriteDistance, bool isPrint, bool usePath, WarpingPathTypeItem **pathList);

/**
*TASK:to compute the angle of rotation by the data of gyroscope
*start:the start position of the recognized subsequence in queue
*end:the end position of the recognized subsequence in queue
*queue:user data sequence
*/
double getDegreeFromGyro(int start, int end, SqQueue *queue);

/**
*TASK:the traditional DTW distance computation
*og:the structure of a defined gesture
*head:a sequence of user data
*/
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
