#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include <process.h>
#include <time.h> //time_t time()
#include <string.h>

#include "DataCalibrator.h"
#include "DataNode.h"
#include "FileUtil.h"
#include "PktParser.h"
#include "SerialPort.h"
#include "SPRING.h"
#include "TargetRecognition.h"
#include "LampCmd.h"

#define MAG_CALI_TIME 20 //the time need to collect initial data, in seconds.

#define INIT_LAMP_HEADING 10 //the time need to collect data for calculating the direction from lamp2 to lamp1

#define TARGET_DATA_NUM 20

const PktData ZERO_PKT = {0.0, 0.0};

//used to only initialize calibrator once.
bool isCalibratorBeingInitialized = false;

bool isCalibratorInitialized = false;

GRProcess *grpHead = NULL;// the list of the custom gestures that have been loaded to the system

/** Initialize calibrator and heading.
 * 1. collect initial data in MAG_CALI_TIME seconds
 * 2. do calibration for initial data
 * 3. if calibrator is invalid, repeat step 1.
 * void initCalibrator(HANDLE hComm) {
 *     printf("\n=========================  Initialize Calibrator  =====================\n");
 *     PktData pktData;
 *     //Mag data list for initialization
 *     DataHeadNode *ptr = create_list_with_head();
 *     int len;
 *
 *     while (true) {
 *         printf("\nCollect initial data in the next %d seconds ...... \n\n", MAG_CALI_TIME);
 *         //Before read, flush the buffer.
 *         purgePort(hComm);
 *
 *         time_t timeBegin = time(NULL);
 *         while (true) {
 *             pktData = blockingReadOnePacket(hComm);
 *             if(equals(pktData, ZERO_PKT)) {
 *                 continue;
 *             }
 *             add_to_list_head(ptr, pktData);
 *
 *             if(time(NULL) - timeBegin >= MAG_CALI_TIME)
 *                 break;
 *         }
 *
 *         //Start prepare double array for calculate calibrator
 *         len = ptr->length;
 *         printf("Initialize data length: %d \n", len);
 *
 *         double magDataX[len] ;
 *         double magDataY[len] ;
 *         double magDataZ[len] ;
 *         double heading[len];
 *         fillMagDataArray(ptr, magDataX, magDataY, magDataZ);
 *
 *         //pass magData to calibrator
 *         calculateCalibrator(magDataX, magDataY, magDataZ, len);
 *
 *         write_list_to_file("C:/Users/weizi/Desktop/Raw_Initial_Data.txt", ptr);
 *         clear_list(ptr);
 *
 *         if(! calibrateMagData(magDataX, magDataY, magDataZ, heading, len))
 *             continue;
 *
 *         write_mag_to_file("C:/Users/weizi/Desktop/Corrected_Initial_Mag_Data.txt", magDataX, magDataY, magDataZ, heading, len);
 *
 *         if(isCalibratorValid(magDataX, magDataY, magDataZ, len))
 *             break;
 *     }
 *     free_list(ptr);
 *     isCalibratorInitialized = true;
 *     printf("\n============================  Initialize Over  ========================\n");
 * }
 *
 * //calculate the heading from lamp 2 to lamp 1.
 * double initHeading(HANDLE hComm) {
 *     PktData pktData;
 *
 *     //the models of the four gestures
 *     char *gestureModel = "./gesture_model/target.txt";
 *     //the tresholds of four model gestures
 *     double threshold = TARGET_THRESHOLD;
 *     //the time limit of four model gestures
 *     double timeLimit = TARGET_TIMELIMIT;
 *
 *     //initialize the four models and their GestureRecognitionProcess
 *     OriginalGesture *og;
 *     GRProcess grp;
 *     og = read_file_to_init_original_gesture(gestureModel);
 *     int m = og->m;
 *     //Pay attention to Free memory !!!
 *     double *distanceArray = (double *)malloc(sizeof(double) * (m + 1));
 *     double *distanceArrayLast = (double *)malloc(sizeof(double) * (m + 1));
 *     int *startArray = (int *)malloc(sizeof(int) * (m + 1));
 *     int *startArrayLast = (int *)malloc(sizeof(int) * (m + 1));
 *     long int *timeArray = (long int *)malloc(sizeof(long int) * (m + 1));
 *     long int *timeArrayLast = (long int *)malloc(sizeof(long int) * (m + 1));
 *     double dmin = DBL_MAX;
 *     int te = 1;
 *     int ts = 1;
 *     int k = 0;
 *     for(k = 0; k <= m; k++) {
 *         distanceArrayLast[k] = DBL_MAX;
 *         startArrayLast[k] = 0;
 *         timeArrayLast[k] = 0;
 *     }
 *     grp.distanceArray = distanceArray;
 *     grp.distanceArrayLast = distanceArrayLast;
 *     grp.dmin = dmin;
 *     grp.originalGesture = *(og);
 *     grp.startArray = startArray;
 *     grp.startArrayLast = startArrayLast;
 *     grp.timeArray = timeArray;
 *     grp.timeArrayLast = timeArrayLast;
 *     grp.threshold = threshold;
 *     grp.te = te;
 *     grp.ts = ts;
 *     grp.times = 0;
 *     grp.times = 0;
 *     grp.type = TARGET_TYPE;
 *     grp.timeLimit = timeLimit;
 *
 *     //the heading from lamp2 to lamp1
 *     double sum;
 *
 *     //Mag data list for calculating angle
 *     SqQueue * queue = create_empty_queue();
 *     int trueNum = 0;
 *     DataHeadNode *targetHead = create_list_with_head();
 *
 *     int len = 50;
 *     while(true) {
 *         printf("\nDetecting the direction in the next %d seconds ...... \n\n", INIT_LAMP_HEADING);
 *
 *         //Before read, flush the buffer.
 *         purgePort(hComm);
 *         time_t timeBegin = time(NULL);
 *         while (true) {
 *             pktData = blockingReadOnePacket(hComm);
 *             purgePort(hComm);
 *             if(equals(pktData, ZERO_PKT)) {
 *                 continue;
 *             }
 *             int i;
 *             //Notice: it will override original raw data if queue is full
 *             //int position = add_to_queue(queue, pktData);
 *             //input the current data into the SPRING
 *             if(SPRING(pktData, &grp,position, queue, 0) == TARGET_TYPE) {
 *                 printf("!!!!!!!!!!!!\n");
 *                 trueNum++;
 *                 add_to_list_head(targetHead, pktData);
 *             }
 *             if(trueNum >= len) {
 *                 trueNum = 0;
 *                 break;
 *             }
 *         }
 *
 *         //Start prepare double array for calculate calibrator
 *         double magDataX[len] ;
 *         double magDataY[len] ;
 *         double magDataZ[len] ;
 *         double heading[len];
 *         fillMagDataArray(targetHead, magDataX, magDataY, magDataZ);
 *
 *         write_list_to_file("C:/Users/xing/Desktop/Raw_Direction_Cal_Data.txt", targetHead);
 *         clear_list(targetHead);
 *
 *         if(! calibrateMagData(magDataX, magDataY, magDataZ, heading, len))
 *             continue;
 *
 *         write_mag_to_file("C:/Users/xing/Desktop/Corrected_Direction_Cal_Mag_Data.txt", magDataX, magDataY, magDataZ, heading, len);
 *         sum = 0.0;
 *         int i;
 *         for(i = 0; i < len; i ++) {
 *             sum += heading[i];
 *         }
 *         sum = sum / len;
 *         printf("\n!!!!!!!!!!!!!!!!!!!!!!2 to 1 direction is ok\n%f!!!!!!!!!!!!!!!!!!\n",sum);
 *         break;
 *     }
 *     //free all list data
 *     free_list(targetHead);
 *     free_queue(queue);
 *     return sum;
 * }
 */

/**
*TASK:ask the user to repeat the gesture and train the threshold and time limit
*ogFileName:the template file name
*hComm:the serial port
*customGestureName:gesture name
*/
GRProcess train_user_template(char *ogFileName, HANDLE hComm, char *customGestureName)
{
    OriginalGesture *og;
    GRProcess grp;
    og = read_file_to_init_original_gesture(ogFileName, true, false,0,NULL);
    int m = og->m;

    OriginalGesture *normalGesture = read_file_to_init_original_gesture("./gesture_model/click.txt",false,false,0,NULL);

    //use the click action to compute the new gesture' s normal distance, and then use this to compute the initial threshold
    int threshold = compute_traditional_DTW(og, normalGesture->head) / 1.3;
    int timeLimit = 200;//userTimeSpan - 1500 > 0 ? userTimeSpan - 1500 : userTimeSpan;
    printf("timeLimit = %d####################threshold = %d\n",timeLimit,threshold);

    //initialize the gesture recognition process structure
    double *distanceArray = (double *)malloc(sizeof(double) * (m + 1));
    double *distanceArrayLast = (double *)malloc(sizeof(double) * (m + 1));
    int *startArray = (int *)malloc(sizeof(int) * (m + 1));
    int *startArrayLast = (int *)malloc(sizeof(int) * (m + 1));
    long int *timeArray = (long int *)malloc(sizeof(long int) * (m + 1));
    long int *timeArrayLast = (long int *)malloc(sizeof(long int) * (m + 1));
    double dmin = DBL_MAX;
    int te = 1;
    int ts = 1;
    int k = 0;
    for(k = 0; k  <= m; k++)
    {
        distanceArrayLast[k] = DBL_MAX;
        startArrayLast[k] = 0;
        timeArrayLast[k] = 0;
    }
    grp.distanceArray = distanceArray;
    grp.distanceArrayLast = distanceArrayLast;
    grp.dmin = dmin;
    grp.originalGesture = *(og);
    grp.startArray = startArray;
    grp.startArrayLast = startArrayLast;
    grp.timeArray = timeArray;
    grp.timeArrayLast = timeArrayLast;
    grp.threshold = threshold;
    grp.te = te;
    grp.ts = ts;
    grp.times = 0;
    grp.times = 0;
    grp.type = CUSTOM_TYPE;
    grp.timeLimit = timeLimit;
    grp.name = customGestureName;

    printf("please repeat the gesture for at least six times!!\n\n");

    SqQueue * queue = create_empty_queue();
    PktData pktData;
    int customNum = 0;
    double thresholdArray[m * 5];
    int timeSpanArray[m * 5];
    int springType = NONE_TYPE;
    int dataNum = grp.originalGesture.m > 70 ?  grp.originalGesture.m * 5 * 2 : grp.originalGesture.m * 5 * 4;

    char trainFileName[60];
    sprintf(trainFileName, "./%d_train.txt", getLocalTime());

    DataHeadNode *trainHead = create_list_with_head();

    //Before read, flush the buffer.
    purgePort(hComm);
    int i = 0;
    for(i = 0; i < dataNum; i ++)
    {
        pktData = blockingReadOnePacket(hComm);
        if(equals(pktData, ZERO_PKT))
        {
            i--;

            continue;
        }
        pktData.pktNumber = i + 1;
        int position = add_to_queue(queue, pktData);

        add_to_list_end(trainHead, pktData);

        //input the current data into the SPRING
        springType = SPRING(pktData, &grp, position, queue, false, false, false, false, NULL);

        if(springType == CUSTOM_TYPE)
        {
            //record the recognized gesture parameters
            thresholdArray[customNum] = grp.distanceArray[m];
            timeSpanArray[customNum] = grp.timee - grp.times;
            customNum++;
        }
    }
    free_list(trainHead);

    //sort all the thresholds in the repeated period
    for(i = 0; i < customNum; i++)
    {
        int j = i;
        for(; j < customNum - 1; j++)
        {
            if(thresholdArray[j] > thresholdArray[j + 1])
            {
                double tmp = thresholdArray[j + 1];
                thresholdArray[j + 1] = thresholdArray[j];
                thresholdArray[j] = tmp;
            }

            if(timeSpanArray[j] < timeSpanArray[j + 1])
            {
                int tmp = timeSpanArray[j + 1];
                timeSpanArray[j + 1] = timeSpanArray[j];
                timeSpanArray[j] = tmp;
            }
        }
    }

    //use the five least distance to compute the appropriate threshold
    double thresholdTrue = 0;
    int timeSpanTrue = 0;
    for(i = 0; i < 5; i++)
    {
        thresholdTrue += thresholdArray[i];
        timeSpanTrue += (timeSpanArray[i] > 4000 ? 4000 : timeSpanArray[i]);
        printf("\n\nthe %i th  gesture choose, threshold = %f, timespan = %d\n",i,thresholdArray[i],timeSpanArray[i]);
    }

    //compute the appropriate parameters
    thresholdTrue /= 5;
    timeSpanTrue /= 5;
    grp.timeLimit = timeSpanTrue * 0.6;//make the time limit more available
    grp.threshold = thresholdTrue * 1.2;//make the threshold more available

    char paraFileName[90];
    sprintf(paraFileName, "./custom_gesture/%s_parameter.txt",customGestureName);
    save_user_template_parameter(grp.threshold, grp.timeLimit, paraFileName);

    printf("train successfully!\nthreshold = %f\ntimespan = %d\n",grp.threshold,grp.timeLimit);

    free_queue(queue);
    dmin = DBL_MAX;
    te = 1;
    ts = 1;
    grp.dmin = dmin;
    grp.te = te;
    grp.ts = ts;
    grp.times = 0;
    grp.timee = 0;
    WarpingPathList *warpingPathList = (WarpingPathList*) malloc(sizeof(WarpingPathList));
    warpingPathList->lengthY = m;
    warpingPathList->position = -1;
    warpingPathList->next = NULL;
    warpingPathList->pre = NULL;
    grp.warpingPathMetrixHead.head = warpingPathList;
    grp.warpingPathMetrixHead.tail = warpingPathList;
    grp.warpingPathMetrixHead.headNum = -1;
    grp.warpingPathMetrixHead.length = 1;
    for(k = 0; k  <= m; k++)
    {
        grp.distanceArrayLast[k] = DBL_MAX;
        grp.startArrayLast[k] = 0;
        grp.timeArrayLast[k] = 0;
    }

    return grp;
}

/**
*TASK:the main process of create a new custom gesture
*/
void collect_template(HANDLE hComm)
{
    SqQueue * queue = create_empty_queue();

    char rawDataFileName[60];  			//The file stores raw data
    char correctedDataFileName[60];  	//The file stores corrected magnetic data

    //the models of the four gestures
    char *gestureModel[DTW_NUM + 1] = {"./gesture_model/target.txt","./gesture_model/point.txt",
                                "./gesture_model/rotate_right_half.txt","./gesture_model/rotate_right_full.txt",
                                "./gesture_model/rotate_left_half.txt","./gesture_model/rotate_left_full.txt",
                                "./gesture_model/slide_over.txt","./activity_model/stand_up.txt",
                                "./activity_model/sit_down.txt","./activity_model/walk.txt", "./gesture_model/click.txt"
                                      };

    //the tresholds of four model gestures
    double threshold[DTW_NUM + 1] = {TARGET_THRESHOLD,POINT_THRESHOLD,ROTATE_RIGHT_HALF_THRESHOLD,
                        ROTATE_RIGHT_FULL_THRESHOLD,ROTATE_LEFT_HALF_THRESHOLD,ROTATE_LEFT_FULL_THRESHOLD,
                        SLIDE_OVER_THRESHOLD,STAND_UP_THRESHOLD,SIT_DOWN_THRESHOLD,WALK_THRESHOLD,CLICK_THRESHOLD
                                    };

    //the time limit of four model gestures
    double timeLimit[DTW_NUM + 1] = {TARGET_TIMELIMIT,POINT_TIMELIMIT,ROTATE_RIGHT_HALF_TIMELIMIT,
                        ROTATE_RIGHT_FULL_TIMELIMIT,ROTATE_LEFT_HALF_TIMELIMIT,ROTATE_LEFT_FULL_TIMELIMIT,
                        SLIDE_OVER_TIMELIMIT,STAND_UP_TIMELIMIT,SIT_DOWN_TIMELIMIT,WALK_TIMELIMIT,CLICK_TIMELIMIT
                                    };

    //initialize the four models and their GestureRecognitionProcess
    //the order is :
    //1->point
    //2->rotate right
    //3->rotate left
    //4->slide over
    OriginalGesture *og[DTW_NUM + 1];
    GRProcess grp[DTW_NUM + 1];
    int gt = 0;
    for(gt = 0; gt < DTW_NUM + 1; gt++)
    {
        og[gt] = read_file_to_init_original_gesture(gestureModel[gt],false,false,0,NULL);
        int m = og[gt]->m;
        //Pay attention to Free memory !!!
        double *distanceArray = (double *)malloc(sizeof(double) * (m + 1));
        double *distanceArrayLast = (double *)malloc(sizeof(double) * (m + 1));
        int *startArray = (int *)malloc(sizeof(int) * (m + 1));
        int *startArrayLast = (int *)malloc(sizeof(int) * (m + 1));
        long int *timeArray = (long int *)malloc(sizeof(long int) * (m + 1));
        long int *timeArrayLast = (long int *)malloc(sizeof(long int) * (m + 1));
        double dmin = DBL_MAX;
        int te = 1;
        int ts = 1;
        int k = 0;
        for(k = 0; k <= m; k++)
        {
            distanceArrayLast[k] = DBL_MAX;
            startArrayLast[k] = 0;
            timeArrayLast[k] = 0;
        }
        grp[gt].distanceArray = distanceArray;
        grp[gt].distanceArrayLast = distanceArrayLast;
        grp[gt].dmin = dmin;
        grp[gt].originalGesture = *(og[gt]);
        grp[gt].startArray = startArray;
        grp[gt].startArrayLast = startArrayLast;
        grp[gt].timeArray = timeArray;
        grp[gt].timeArrayLast = timeArrayLast;
        grp[gt].threshold = threshold[gt];
        grp[gt].te = te;
        grp[gt].ts = ts;
        grp[gt].times = 0;
        grp[gt].times = 0;
        grp[gt].type = gt;
        grp[gt].timeLimit = timeLimit[gt];
    }

    char customGestureName[60];
    int functionNum;
    printf("!!please input the name of the gesture you want to create!!\n");
    scanf("%s",customGestureName);

    printf("!!please input the function of the gesture you want to create!!\n");
    printf("0:turn on or off\n");
    printf("1:brightness up\n");
    printf("2:brightness down\n");
    printf("3:hue up\n");
    printf("4:bue down\n");

    scanf("%d",&functionNum);

    //Before read, flush the buffer.
    purgePort(hComm);

    int trueNum = 0;
    int springType = NONE_TYPE;
    bool isRecord = false;
    DataHeadNode *targetHead = create_list_with_head();

    bool hasCustom = false;
    GRProcess customGRP;
    PktData pktData;
    int i;
    printf("\nstart the template with shacking the MotionNet!!!\n");
    while(true)
    {
        pktData = blockingReadOnePacket(hComm);
        if(equals(pktData, ZERO_PKT))
        {
            continue;
        }
        //Notice: it will override original raw data if queue is full
        int position = add_to_queue(queue, pktData);

        if(hasCustom == false)
        {
            //input the current data into the SPRING
            int l = 0;
            for(l = DTW_NUM; l <= DTW_NUM; l++)
            {
                springType = SPRING(pktData, &grp[l],position, queue, false, false, false, false, NULL);
            }

            if(isRecord == true)
            {
                add_to_list_end(targetHead, pktData);
            }

            if(springType == CLICK_TYPE)
            {
                if(isRecord == false)
                {
                    isRecord = true;
                }
                else if(isRecord == true)
                {
                    isRecord = false;
                    printf("saving the template.....\n");
                    //delete the click data from the list and then output the list to file
                    long timeStampTmp = queue->timeStamp[(grp[CLICK_TYPE].ts - 20 + MAX_SIZE - 1) % (MAX_SIZE - 1)];

                    DataNode *head = targetHead->head;
                    while(head->next != NULL)
                    {
                        if(head->next->packetData.timeStamp == timeStampTmp)
                        {
                            DataNode *tmpList = head->next;
                            //free_list(tmpList);
                            head->next = NULL;
                            break;
                        }
                        head = head->next;
                    }
                    //printf("going to save!\n");
                    char templateFileName[90];
                    sprintf(templateFileName, "./custom_gesture/%s_template.txt",customGestureName);

                    //remove the useless data at the beginning
                    int j = 0;
                    for(j = 0; j < 10; j++)
                    {
                        targetHead->head = targetHead->head->next;
                    }
                    save_user_template(templateFileName, targetHead);

                    int userTimeSpan = targetHead->tail->packetData.timeStamp - targetHead->head->packetData.timeStamp;

                    clear_list(targetHead);
                    printf("a template has been saved!\n");

                    //train the gesture
                    customGRP = train_user_template(templateFileName,hComm,customGestureName);

                    clear_queue(queue);

                    hasCustom = true;

                    break;
                }
            }
        }
    }

    printf("\n\ninput the number of the target you want to recognize:\n");
    int magNum;
    scanf("%d",&magNum);
    int magNumTmp = 0;

    //Before read, flush the buffer.
    purgePort(hComm);

    //collect the mag templates of the gesture
    WarpingPathTypeItem *pathList = (WarpingPathTypeItem *) malloc(sizeof(WarpingPathTypeItem *));
    while(magNumTmp < magNum)
    {
        pktData = blockingReadOnePacket(hComm);
        if(equals(pktData, ZERO_PKT))
        {
            continue;
        }

        //Notice: it will override original raw data if queue is full
        int position = add_to_queue(queue, pktData);

        int springType = SPRING(pktData,&customGRP,position,queue, false, false, false, true, &pathList);

        if(springType == CUSTOM_TYPE)
        {
            /**normalize the mag-----queue,customGRP.originalGesture.head,customGRP.ts,customGRP.te,pathList*/
            AverageList *normalizedTmpData = Normalization(pathList,queue,customGRP.ts,customGRP.te);
            /**save them----customGRP.originalGesture.magListHead*/

            char magTempFileName[100];
            sprintf(magTempFileName, "./custom_gesture/%s_magTemplate_%d.txt", customGestureName,magNumTmp);
            printf("save %s 's %d th mag template\n",magTempFileName, magNumTmp);
            save_mag_template(magTempFileName,normalizedTmpData);

            magNumTmp++;
        }
    }
    CustomGestureItem *item = (CustomGestureItem*) malloc(sizeof(CustomGestureItem));
    item->gestureFunction = functionNum;
    item->gestureName = customGestureName;
    item->magNum = magNum;

    insert_new_custom_gesture_item(*item);

    int magLen = get_queue_length(queue);

    double heading[magLen];

    write_queue_to_file(rawDataFileName, queue);

    calibrateMagData(queue->magXData, queue->magYData, queue->magZData, heading, magLen);

    write_mag_to_file(correctedDataFileName, queue->magXData, queue->magYData, queue->magZData, heading, magLen);

    free_queue(queue);
    free_list(targetHead);

    //show_main_menu(hComm);
}

/**
*TASK:the main body of the control part
*/
void control_lamp(HANDLE hComm)
{
    GRProcess *tmpGRP = grpHead;
    if(tmpGRP == NULL)
    {
        printf("no gesture is loaded!!!please load gesture first!!\n");
        return;
    }
    PktData pktData;

    //Before read, flush the buffer.
    purgePort(hComm);
    SqQueue * queue = create_empty_queue();
    while(true)
    {
        tmpGRP = grpHead;
        pktData = blockingReadOnePacket(hComm);
        if(equals(pktData, ZERO_PKT))
        {
            continue;
        }

        //Notice: it will override original raw data if queue is full
        int position = add_to_queue(queue, pktData);
        WarpingPathTypeItem *pathList;
        int springType;
        while(tmpGRP != NULL)
        {
            springType = SPRING(pktData,tmpGRP,position,queue, false, false, false, true, &pathList);
            if(springType == CUSTOM_TYPE)
            {
                /**normalize the mag---queue,tmpGRP->originalGesture.head,tmpGRP->ts,tmpGRP->te,pathList*/
                AverageList *normalizedUserData = Normalization(pathList, queue, tmpGRP->ts, tmpGRP->te);

                DataHeadNode *tmpMagList;
                tmpMagList = tmpGRP->originalGesture.magListHead;
                int k = 0;
                double magDistanceArray[tmpGRP->originalGesture.magNum];
                double magDisMin = DBL_MAX;
                int targetNum = 0;
                for(k = 0; k < tmpGRP->originalGesture.magNum; k++)
                {
                    /**compute the mag distance---tmpMagList*/
                    magDistanceArray[k] = compute_magdata_distance(normalizedUserData, tmpMagList) ;
                    printf("\n\nk=%d;distance=%lf\n", k, magDistanceArray[k]);

                    //compute the least mag distance one to be the target
                    if(magDistanceArray[k] < magDisMin)
                    {
                        magDisMin = magDistanceArray[k];
                        targetNum = k + 1;
                    }
                    tmpMagList = tmpMagList->next;
                }
                createCommand(tmpGRP->functionNum,targetNum);

            }
            tmpGRP = tmpGRP->next;

        }
    }
    free_queue(queue);
}

/**
*TASK:show the list of all custom gestures, and load gestures user select and initialize the GRP
*/
void show_load_custom_gesture(HANDLE hComm)
{
    GRProcess *grpTmp = NULL;
    bool isFirst = true;
    bool backToMain = false;
    while(true)
    {
        if(backToMain)
            break;

        printf("\n\n\n\n\n------------load custom gesture menu----------\n");
        printf("1. back to main menu\n");

        //read the custom gesture list and print them
        CustomGestureList *list = (CustomGestureList*) malloc(sizeof(CustomGestureList));
        load_custom_gesture_list(list);
        int i = 0;
        CustomGestureItem *p = list->head;

        for(i = 0; i < list->length; i++)
        {
            printf("%d. %s---function num : %d---target num : %d\n", i + 2, p->gestureName, p->gestureFunction, p->magNum);

            p = p->next;
        }

        printf("already loaded : ");
        GRProcess *tmptmpgrp = grpHead;
        while(tmptmpgrp != NULL)
        {
            printf("%s, ",tmptmpgrp->name);
            tmptmpgrp = tmptmpgrp->next;
            printf("\n");
        }
        printf("\n");

        int select;
        scanf("%d",&select);
        switch(select)
        {
        case 1:
        {
            if(grpTmp != NULL)
            {
                grpTmp->next = NULL;
            }
            backToMain = true;
            break;
        }
        default:
        {
            /**load the selected gesture*/
            int j = 0;
            p = list->head;
            for(j = 1; j < select - 1; j++)
            {
                p = p->next;
            }
            printf("name-------%s",p->gestureName);
            char templateFileName[60],parameterFileName[60];
            sprintf(templateFileName, "./custom_gesture/%s_template.txt" ,p->gestureName);
            sprintf(parameterFileName, "./custom_gesture/%s_parameter.txt" ,p->gestureName);
            OriginalGesture *og;
            GRProcess *grp = (GRProcess*) malloc(sizeof(GRProcess));
            og = read_file_to_init_original_gesture(templateFileName,true,true,p->magNum,p->gestureName);
            og->magNum = p->magNum;
            CustomGestureParameter cgparam = read_custom_gesture_parameter(parameterFileName);
            int m = og->m;

            double *distanceArray = (double *)malloc(sizeof(double) * (m + 1));
            double *distanceArrayLast = (double *)malloc(sizeof(double) * (m + 1));
            int *startArray = (int *)malloc(sizeof(int) * (m + 1));
            int *startArrayLast = (int *)malloc(sizeof(int) * (m + 1));
            long int *timeArray = (long int *)malloc(sizeof(long int) * (m + 1));
            long int *timeArrayLast = (long int *)malloc(sizeof(long int) * (m + 1));
            WarpingPathList *warpingPathList = (WarpingPathList*) malloc(sizeof(WarpingPathList));
            warpingPathList->lengthY = m;
            warpingPathList->position = -1;
            warpingPathList->next = NULL;
            warpingPathList->pre = NULL;

            double dmin = DBL_MAX;
            int te = 1;
            int ts = 1;
            int k = 0;
            for(k = 0; k <= m; k++)
            {
                distanceArrayLast[k] = DBL_MAX;
                startArrayLast[k] = 0;
                timeArrayLast[k] = 0;
            }
            grp->distanceArray = distanceArray;
            grp->distanceArrayLast = distanceArrayLast;
            grp->dmin = dmin;
            grp->originalGesture = *(og);
            grp->startArray = startArray;
            grp->startArrayLast = startArrayLast;
            grp->timeArray = timeArray;
            grp->timeArrayLast = timeArrayLast;
            grp->threshold = cgparam.threshold;
            grp->te = te;
            grp->ts = ts;
            grp->times = 0;
            grp->times = 0;
            grp->type = CUSTOM_TYPE;
            grp->timeLimit = cgparam.timeSpan;
            grp->name = p->gestureName;
            grp->functionNum = p->gestureFunction;
            grp->warpingPathMetrixHead.head = warpingPathList;
            grp->warpingPathMetrixHead.tail = warpingPathList;
            grp->warpingPathMetrixHead.headNum = -1;
            grp->warpingPathMetrixHead.length = 1;

            if(isFirst)
            {
                isFirst = false;
                grpHead = grp;
            }
            else
            {
                grpTmp->next = grp;
            }
            grpTmp = grp;
            grp->next = NULL;
            break;
        }
        }
    }
}

/**
*TASK:show the main menu
*/
void show_main_menu(HANDLE hComm)
{
    while(true)
    {
        printf("\n\n\n\n\n------------main menu---------\n");
        printf("1.start control system\n");
        printf("2.create custom gesture\n");
        printf("3.load custom gestures\n");

        int selectItem;
        scanf("%d",&selectItem);

        switch(selectItem)
        {
        case 1:
            control_lamp(hComm);
            break;/**start the thread to run control lamp*/
        case 2:
            collect_template(hComm);
            break;/**run the custom gesture creating function*/
        case 3:
            show_load_custom_gesture(hComm);
            break;
        }
    }

}

/**
*the main body of the thread
*/
void ThreadFunc(Params* params)
{
    int target;
    double headingFrom2To1;
    printf("======== SubThread %ld is watching over %s port ===========\n", GetCurrentThreadId(), params->gszPort);

    HANDLE hComm = openPort(params->gszPort);
    if (hComm == INVALID_HANDLE_VALUE)
    {
        printf("failed to open serial port %s \n", params->gszPort);
        return;
    }
    else
    {
        if (setupPort(hComm))
        {
            // all sensors use the same calibration matrix and offset
            show_main_menu(hComm);
        }
        closePort(hComm);
    }
}

int main(int argc, char *argv[])
{
    /** Initialize Hue lamp system */
    if(getBridgeIP() == 1)
    {
        printf("get bridge IP succeed\n");
    }

    if(getUserName("{\"devicetype\":\"my_hue_app#iphone peter\"}"))
    {
        printf("get username succeed\n");
    }

    // Now we only have a wrist sensor, we can also add thigh sensor in the future.
    int portCount = 1;

    HANDLE handle[portCount];
    Params params[portCount];

    int sensorType[2] = {WRIST_TYPE,THIGH_TYPE};

    int portId;     //e.g com#3 's portId is 3.
    int i;
    for(i = 0; i < portCount; i ++)
    {
        Params param;
        printf("input the bluetooth port COM number: \n");
        scanf("%d", &portId);
        sprintf(param.gszPort, "\\\\.\\com%d" ,portId);

        //printf("Input the count of mag data need to be collected this time :\n");
        //scanf("%d", &param.magDataNum);

        param.sensorType = sensorType[i];
        params[i] = param;
    }
    for(i = 0; i < portCount; i ++)
        handle[i] = (HANDLE) _beginthreadex(NULL, 0, ThreadFunc, &(params[i]), 0, NULL);

    //wait until all sub threads end
    WaitForMultipleObjects(portCount, handle, TRUE, INFINITE);

    clearCalibrator();

    system("pause");
    return 0;
}
