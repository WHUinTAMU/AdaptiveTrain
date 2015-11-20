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

//1. collect initial data in MAG_CALI_TIME seconds
//2. do calibration for initial data
//3. if calibrator is invalid, repeat step 1.
/* void initCalibrator(HANDLE hComm) {
 *     printf("\n=========================  Initialize Calibrator  =====================\n");
 *     PktData pktData;
 *
 *     //Mag data list for initialization
 *     DataHeadNode *ptr = create_list_with_head();
 *     int len;
 *
 *     while (true) {
 *         printf("\nCollect initial data in the next %d seconds ...... \n\n", MAG_CALI_TIME);
 *
 *         //Before read, flush the buffer.
 *         purgePort(hComm);
 *
 *         time_t timeBegin = time(NULL);
 *
 *         while (true) {
 *             pktData = blockingReadOnePacket(hComm);
 *
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
 *
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
 *
 *     //free all list data
 *     free_list(ptr);
 *     isCalibratorInitialized = true;
 *     printf("\n============================  Initialize Over  ========================\n");
 * }
 *
 *
 * //calculate the heading from lamp 2 to lamp 1.
 * double initHeading(HANDLE hComm) {
 *     PktData pktData;
 *
 *     //the models of the four gestures
 *     char *gestureModel = "./gesture_model/target.txt";
 *
 *     //the tresholds of four model gestures
 *     double threshold = TARGET_THRESHOLD;
 *
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
 *
 *         time_t timeBegin = time(NULL);
 *
 *         while (true) {
 *             pktData = blockingReadOnePacket(hComm);
 *             purgePort(hComm);
 *             if(equals(pktData, ZERO_PKT)) {
 *                 continue;
 *             }
 *
 *             int i;
 *
 *             //Notice: it will override original raw data if queue is full
 *             //int position = add_to_queue(queue, pktData);
 *
 *             //input the current data into the SPRING
 *             if(SPRING(pktData, &grp,position, queue, 0) == TARGET_TYPE) {
 *                 printf("!!!!!!!!!!!!\n");
 *                 trueNum++;
 *                 add_to_list_head(targetHead, pktData);
 *             }
 *
 *             if(trueNum >= len) {
 *
 *                 trueNum = 0;
 *                 break;
 *
 *             }
 *         }
 *
 *         //Start prepare double array for calculate calibrator
 *
 *         double magDataX[len] ;
 *         double magDataY[len] ;
 *         double magDataZ[len] ;
 *         double heading[len];
 *
 *         fillMagDataArray(targetHead, magDataX, magDataY, magDataZ);
 *
 *         write_list_to_file("C:/Users/xing/Desktop/Raw_Direction_Cal_Data.txt", targetHead);
 *         clear_list(targetHead);
 *
 *         if(! calibrateMagData(magDataX, magDataY, magDataZ, heading, len))
 *             continue;
 *
 *         write_mag_to_file("C:/Users/xing/Desktop/Corrected_Direction_Cal_Mag_Data.txt", magDataX, magDataY, magDataZ, heading, len);
 *
 *         //consider variance
 *
 *         sum = 0.0;
 *         int i;
 *         for(i = 0; i < len; i ++) {
 *             sum += heading[i];
 *         }
 *
 *         sum = sum / len;
 *
 *         printf("\n!!!!!!!!!!!!!!!!!!!!!!2 to 1 direction is ok\n%f!!!!!!!!!!!!!!!!!!\n",sum);
 *         break;
 *     }
 *
 *     //free all list data
 *     free_list(targetHead);
 *     free_queue(queue);
 *     return sum;
 * }
 */

GRProcess trian_user_template(int userTimeSpan, char *templateFileName, HANDLE hComm)
{
    OriginalGesture *og;
    GRProcess grp;
    og = read_file_to_init_original_gesture(templateFileName);
    int m = og->m;

    OriginalGesture *normalGesture = read_file_to_init_original_gesture("./gesture_model/click.txt");
    int threshold = compute_traditional_DTW(og, normalGesture->head) / 1.5;
    int timeLimit = 0;userTimeSpan - 1000 > 0 ? userTimeSpan - 1000 : userTimeSpan;
    printf("timeLimit = %d####################threshold = %d\n",timeLimit,threshold);

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
    for(k = 0; k  <= m; k++) {
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

    printf("please repeat the gesture for at least six times!!\n\n");

    SqQueue * queue = create_empty_queue();
    PktData pktData;
    int customNum = 0;
    double thresholdArray[m * 5];
    int timeSpanArray[m * 5];
    int springType = NONE_TYPE;
    int dataNum = grp.originalGesture.m * 5 * 1.5;

    char trainFileName[60];
    sprintf(trainFileName, "./%d_train.txt"
            ,getLocalTime());

    DataHeadNode *trainHead = create_list_with_head();

    //Before read, flush the buffer.
    purgePort(hComm);
    int i = 0;
    for(i = 0; i < dataNum; i ++) {
        pktData = blockingReadOnePacket(hComm);
        if(equals(pktData, ZERO_PKT)) {
            i--;
            printf("******************************\n");
            continue;
        }
        pktData.pktNumber = i + 1;
        int position = add_to_queue(queue, pktData);
//        printf("%f\t%f\t%f\t%f\t%f\t%f\t%d\n", pktData.accX, pktData.accY, pktData.accZ,
//            pktData.gyroX, pktData.gyroY, pktData.gyroZ, pktData.pktNumber);
        //write_pkt_to_file(stream, pktData);
        add_to_list_end(trainHead, pktData);

        //input the current data into the SPRING
        springType = SPRING(pktData, &grp, position, queue, false, true, true);
        //printf("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%d\n",springType);

        if(springType == CUSTOM_TYPE)
        {
            thresholdArray[customNum] = grp.distanceArray[m];
            timeSpanArray[customNum] = grp.timee - grp.times;
            customNum++;
        }
    }
    write_list_to_file(trainFileName, trainHead);
    free_list(trainHead);

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

    double thresholdTrue = 0;
    int timeSpanTrue = 0;
    for(i = 0; i < 5; i++)
    {
        thresholdTrue += thresholdArray[i];
        timeSpanTrue += (timeSpanArray[i] > 4000 ? 4000 : timeSpanArray[i]);
        printf("$$$$$$$$$$$$$$%i$$$$$$threshold = %f::::timespan = %d\n",i,thresholdArray[i],timeSpanArray[i]);
    }

    thresholdTrue /= 5;
    timeSpanTrue /= 5;
    grp.timeLimit = timeSpanTrue - 400 > 100 ? timeSpanTrue - 400 : timeSpanTrue;
    grp.threshold = thresholdTrue + 70;

    printf("train successfully!!!!threshold = %f\ntimespan = %d\n",grp.threshold,grp.timeLimit);

    free_queue(queue);
    return grp;
}

void ThreadFunc(Params* params) {
    int target;
    double headingFrom2To1;
    printf("======== SubThread %ld is watching over %s port ===========\n", GetCurrentThreadId(), params->gszPort);

    HANDLE hComm = openPort(params->gszPort);
    if (hComm == INVALID_HANDLE_VALUE) {
        printf("failed to open serial port %s \n", params->gszPort);
        return;
    } else {
        if (setupPort(hComm)) {
            // all sensors use the same calibration matrix and offset


            printf("======================= collect test data for %s =================== \n", params->gszPort);

            SqQueue * queue = create_empty_queue();

            char rawDataFileName[60];  			//The file stores raw data
            char correctedDataFileName[60];  	//The file stores corrected magnetic data

            sprintf(rawDataFileName, "C:/Users/xing/Desktop/%s_Raw_Mag_Data.txt",params->gszPort);
            sprintf(correctedDataFileName, "C:/Users/xing/Desktop/%s_Corrected_Mag_Data.txt",params->gszPort);

            //the models of the four gestures
				char *gestureModel[DTW_NUM + 1] = {"./gesture_model/target.txt"
                    ,"./gesture_model/point.txt"
                    ,"./gesture_model/rotate_right_half.txt"
                    ,"./gesture_model/rotate_right_full.txt"
                    ,"./gesture_model/rotate_left_half.txt"
                    ,"./gesture_model/rotate_left_full.txt"
                    ,"./gesture_model/slide_over.txt"
                    ,"./activity_model/stand_up.txt"
                    ,"./activity_model/sit_down.txt"
                    ,"./activity_model/walk.txt"
                    ,"./gesture_model/click.txt"};

                //the tresholds of four model gestures
                double threshold[DTW_NUM + 1] = {TARGET_THRESHOLD,POINT_THRESHOLD,ROTATE_RIGHT_HALF_THRESHOLD
                ,ROTATE_RIGHT_FULL_THRESHOLD,ROTATE_LEFT_HALF_THRESHOLD,ROTATE_LEFT_FULL_THRESHOLD
                ,SLIDE_OVER_THRESHOLD,STAND_UP_THRESHOLD,SIT_DOWN_THRESHOLD,WALK_THRESHOLD,CLICK_THRESHOLD};

                //the time limit of four model gestures
                double timeLimit[DTW_NUM + 1] = {TARGET_TIMELIMIT,POINT_TIMELIMIT,ROTATE_RIGHT_HALF_TIMELIMIT
                ,ROTATE_RIGHT_FULL_TIMELIMIT,ROTATE_LEFT_HALF_TIMELIMIT,ROTATE_LEFT_FULL_TIMELIMIT
                ,SLIDE_OVER_TIMELIMIT,STAND_UP_TIMELIMIT,SIT_DOWN_TIMELIMIT,WALK_TIMELIMIT,CLICK_TIMELIMIT};

            //initialize the four models and their GestureRecognitionProcess
            //the order is :
            //1->point
            //2->rotate right
            //3->rotate left
            //4->slide over
            OriginalGesture *og[DTW_NUM + 1];
            GRProcess grp[DTW_NUM + 1];
            int gt = 0;
            for(gt = 0; gt < DTW_NUM + 1; gt++) {
                og[gt] = read_file_to_init_original_gesture(gestureModel[gt]);
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
                for(k = 0; k <= m; k++) {
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
            for(i = 0; i < params->magDataNum; i ++) {
                pktData = blockingReadOnePacket(hComm);
                if(equals(pktData, ZERO_PKT)) {
                    continue;
                }
                //Notice: it will override original raw data if queue is full
                int position = add_to_queue(queue, pktData);

                if(hasCustom == false)
                {
                    //input the current data into the SPRING
                    int l = 0;
                    for(l = DTW_NUM; l <= DTW_NUM; l++) {
                        springType = SPRING(pktData, &grp[l],position, queue, false, false, false);
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
                            long timeStampTmp = queue->timeStamp[grp[CLICK_TYPE].ts + 20];
                            printf("end time:%d\n",timeStampTmp);

                            DataNode *head = targetHead->head;
                            while(head->next != NULL)
                            {
                                printf("%d:::::%d\n",timeStampTmp,head->next->packetData.timeStamp);
                                if(head->next->packetData.timeStamp == timeStampTmp)
                                {
                                    DataNode *tmpList = head->next;
                                    //free_list(tmpList);
                                    head->next = NULL;
                                    break;
                                }
                                head = head->next;
                            }
                            printf("going to save!\n");
                            char templateFileName[60];

                            sprintf(templateFileName, "C:\\Users\\weizi\\Desktop\\MotionNetProjectC\\RealTimeGestureRecognition\\%d_template.txt",getLocalTime());
                            save_user_template(templateFileName, targetHead);

                            int userTimeSpan = targetHead->tail->packetData.timeStamp - targetHead->head->packetData.timeStamp;
                            printf("start time = %d ::: end time = %d",targetHead->head->packetData.timeStamp,targetHead->tail->packetData.timeStamp);

                            clear_list(targetHead);
                            printf("a template has been saved!\n");

                            customGRP = trian_user_template(userTimeSpan,templateFileName, hComm);

                            clear_queue(queue);

                            hasCustom = true;
                        }
                    }
                }
                else if(hasCustom == true)
                {
                    springType = SPRING(pktData, &customGRP,position, queue, false, false, true);
                }
            }

            int magLen = get_queue_length(queue);

            double heading[magLen];

            write_queue_to_file(rawDataFileName, queue);

            calibrateMagData(queue->magXData, queue->magYData, queue->magZData, heading, magLen);

            write_mag_to_file(correctedDataFileName, queue->magXData, queue->magYData, queue->magZData, heading, magLen);

            printf("\nSee %s \nand %s\nfor more detail!\n\n",rawDataFileName, correctedDataFileName);

            free_queue(queue);
            free_list(targetHead);
        }
        closePort(hComm);
    }
}

int main(int argc, char *argv[]) {
    /*if(getBridgeIP() == 1)
    {
		printf("get bridge IP succeed\n");
    }

	if(getUserName("{\"devicetype\":\"my_hue_app#iphone peter\"}"))
    {
    	printf("get username succeed\n");
    }*/

    printf("how many sensors do you have :\n");
    int portCount = 0;
    scanf("%d", &portCount);

    HANDLE handle[portCount];
    Params params[portCount];

    //Consider enum
    int sensorType[2] = {WRIST_TYPE,THIGH_TYPE};

    int portId;     //e.g Com#3 's portId is 3.
    int i;
    for(i = 0; i < portCount; i ++) {
        Params param;

        printf("input the port COM number: \n");
        scanf("%d", &portId);
        sprintf(param.gszPort, "\\\\.\\com%d" ,portId);

        printf("Input the count of mag data need to be collected this time :\n");
        scanf("%d", &param.magDataNum);

        param.sensorType = sensorType[i];
        params[i] = param;
        //printf("Param %d : %s,  %d \n", i, params[i].gszPort, params[i].magDataNum);
    }
    for(i = 0; i < portCount; i ++)
        handle[i] = (HANDLE) _beginthreadex(NULL, 0, ThreadFunc, &(params[i]), 0, NULL);

    //wait until all sub threads end
    WaitForMultipleObjects(portCount, handle, TRUE, INFINITE);

    clearCalibrator();

    system("pause");
    return 0;
}



