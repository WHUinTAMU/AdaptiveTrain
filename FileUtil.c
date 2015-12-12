#include "FileUtil.h"

void write_queue_to_file(char * fileName, SqQueue * queue) {
    if(is_empty_queue(queue))
        return;
    else {
        FILE *stream = fopen(fileName, "w+");
        fprintf(stream, "Number\tAccX\t\tAccY\t\tAccZ\t\tGyroX\t\tGyroY\t\tGyroZ\t\tMagX\t\tMagY\t\tMagZ\r\n");

        int i = queue->front;
        while(i != queue->rear) {
            fprintf(stream, "%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\r\n", i,
                    queue->accXData[i], queue->accYData[i], queue->accZData[i],
                    queue->gyroXData[i], queue->gyroYData[i], queue->gyroZData[i],
                    queue->magXData[i] , queue->magYData[i],  queue->magZData[i]);
            i = (i + 1) % MAX_SIZE;
        }
        fclose(stream);
    }
}

void write_list_to_file(char * fileName, DataHeadNode *pHead) {
    DataNode *ptr = pHead->head;
    if(ptr == NULL)
        return;

    PktData pktData;
    FILE *stream = fopen(fileName, "w+");
    fprintf(stream, "AccX\t\tAccY\t\tAccZ\t\tGyroX\t\tGyroY\t\tGyroZ\t\tMagX\t\tMagY\t\tMagZ\t\tNumber\tTimeStamp\r\n");
    while (ptr != NULL) {
        pktData = ptr->packetData;
        fprintf(stream, "%ld\t%lu\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%d\t%d\t%d\t%d\r\n",
        pktData.pktNumber, pktData.timeStamp,
        pktData.accX, pktData.accY, pktData.accZ,
        pktData.gyroX, pktData.gyroY, pktData.gyroZ,
        pktData.magX, pktData.magY, pktData.magZ,
        pktData.rssiData1, pktData.rssiData2, pktData.rssiData3, pktData.rssiData4
        );
        ptr = ptr->next;
    }
    fclose(stream);
}

void write_pkt_to_file(char * fileName, PktData pktData) {
    FILE *stream = fopen(fileName, "a+");
    fprintf(stream, "%f\t%f\t%f\t%f\t%f\t%f\t%d\n", pktData.accX, pktData.accY, pktData.accZ,
            pktData.gyroX, pktData.gyroY, pktData.gyroZ, pktData.pktNumber);
    fclose(stream);
}

void write_mag_to_file(char * fileName, double x[], double y[], double z[], double heading[], int len) {
    FILE *stream = fopen(fileName, "w+");
    fprintf(stream, "Number\tMagX\t\tMagY\t\tMagZ\t\tHeading\r\n");
    int i;
    for(i = 0; i < len; i ++)
        fprintf(stream, "%d\t%f\t%f\t%f\t%.4f\r\n", i, x[i], y[i], z[i], heading[i]);
    fclose(stream);
}

/**
*TASK:load a gesture from txt file
*fileName
*isMag:whether this txt file contain mag data
*isMagTemlpate:whether need to load mag template file
*magTemplateNum:number of mag template
*gestureName: gesture name, in order to load mag template
*
*/
OriginalGesture *read_file_to_init_original_gesture(char * fileName, bool isMag, bool isMagTemplate, int magTemplateNum, char *gestureName) {
    FILE * fp;
    /* open */
    if ((fp = fopen(fileName,"r")) == NULL) {
        printf("Can't open %s\n",fileName);
        exit(1);
    }

    bool isFirst = true;
    OriginalGesture *og = (OriginalGesture*) malloc(sizeof(OriginalGesture));
    DataNode *tmp = NULL;
    DataNode *head = NULL;
    double tmpArray[9];
    int tmpPktNum = 0;
    int i = 1;
    if(isMag)
    {
        //read the acc, gyro and mag data to initialize the OriginalGesture
        while(EOF != fscanf(fp, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%d", &tmpArray[0], &tmpArray[1], &tmpArray[2], &tmpArray[3],
                        &tmpArray[4], &tmpArray[5], &tmpArray[6], &tmpArray[7], &tmpArray[8], &tmpPktNum)) {
            PktData *pd = (PktData*) malloc(sizeof(PktData));
            pd->accX = tmpArray[0];
            pd->accY = tmpArray[1];
            pd->accZ = tmpArray[2];
            pd->gyroX = tmpArray[3];
            pd->gyroY = tmpArray[4];
            pd->gyroZ = tmpArray[5];
            pd->magX = tmpArray[6];
            pd->magY = tmpArray[7];
            pd->magZ = tmpArray[8];
            pd->pktNumber = tmpPktNum;

            DataNode *dn = (DataNode*) malloc(sizeof(DataNode));
            dn->packetData = *pd;

            if(isFirst) {
                isFirst = false;
                head = dn;
                tmp = dn;
            } else {
                tmp->next = dn;
                tmp = dn;
            }
            i++;

        }
    }
    else
    {
        while(EOF != fscanf(fp, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%d", &tmpArray[0], &tmpArray[1], &tmpArray[2], &tmpArray[3],
                        &tmpArray[4], &tmpArray[5], &tmpPktNum)) {
            PktData *pd = (PktData*) malloc(sizeof(PktData));
            pd->accX = tmpArray[0];
            pd->accY = tmpArray[1];
            pd->accZ = tmpArray[2];
            pd->gyroX = tmpArray[3];
            pd->gyroY = tmpArray[4];
            pd->gyroZ = tmpArray[5];
            pd->pktNumber = tmpPktNum;

            DataNode *dn = (DataNode*) malloc(sizeof(DataNode));
            dn->packetData = *pd;

            if(isFirst) {
                isFirst = false;
                head = dn;
                tmp = dn;
            } else {
                tmp->next = dn;
                tmp = dn;
            }
            i++;

        }
    }
    tmp->next = NULL;

    /* close */
    fclose(fp);

    DataHeadNode *dhn = (DataHeadNode*) malloc(sizeof(DataHeadNode));
    dhn->head = head;
    dhn->length = i - 1;

    og->head = dhn;
    og->m = i - 1;

    i = 0;
    if(isMagTemplate)   //if the gesture has mag template, load them
    {
        DataHeadNode *magListHead;
        DataHeadNode *tmpMagList;
        char magTempFileName[90];
        int j = 0;
        bool isFirstList = true;
        printf("mag template num = %d\n",magTemplateNum);
        for(j = 0; j < magTemplateNum; j++)
        {
            i = 0;
            sprintf(magTempFileName, "./custom_gesture/%s_magTemplate_%d.txt", gestureName,j);
            FILE * fp;
            /* open */
            if ((fp = fopen(magTempFileName,"r")) == NULL) {
                printf("Can't open %s\n",fileName);
                exit(1);
            }


            bool isFirst = true;
            DataHeadNode *ml = (DataHeadNode*) malloc(sizeof(DataHeadNode));
            DataNode *tmp = NULL;
            DataNode *head = NULL;
            int tmpPktNum = 0;
            printf("before read\n");
            while(EOF != fscanf(fp, "%lf\t%lf\t%lf", &tmpArray[0], &tmpArray[1], &tmpArray[2])) {
                PktData *pd = (PktData*) malloc(sizeof(PktData));
                pd->magX = tmpArray[0];
                pd->magY = tmpArray[1];
                pd->magZ = tmpArray[2];

                DataNode *dn = (DataNode*) malloc(sizeof(DataNode));
                dn->packetData = *pd;

                if(isFirst) {
                    isFirst = false;
                    head = dn;
                    tmp = dn;
                } else {
                    tmp->next = dn;
                    tmp = dn;
                }
                i++;
            }
            printf("after read\n");
            ml->head = head;
            if(isFirstList)
            {
                magListHead = ml;
                tmpMagList = ml;
                isFirstList = false;
            }
            else
            {
                tmpMagList->next = ml;
                tmpMagList = ml;
            }
            tmpMagList->length = i;
            /* close */
            fclose(fp);

        }
        printf("after for\n");
        tmpMagList->next = NULL;
        og->magListHead = magListHead;

    }

    return og;
}

/**
*TASK:load the parameter of a gesture(threshold and time limit)
*/
CustomGestureParameter read_custom_gesture_parameter(char * fileName)
{
    FILE * fp;
    /* open */
    if ((fp = fopen(fileName,"r")) == NULL) {
        printf("Can't open %s\n",fileName);
        exit(1);
    }

    CustomGestureParameter * cgp = (CustomGestureParameter*) malloc(sizeof(CustomGestureParameter));
    if(EOF != fscanf(fp, "%lf\t%d",&(cgp->threshold),&(cgp->timeSpan)))
    {
        return *cgp;
    }
    printf("error happend when load custom gesture parameter\n");
    return *cgp;
}

/**
*save user acc and gyro and mag template into txt file
*/
void save_user_template(char * fileName, DataHeadNode *dataHeadNode)
{
    DataNode *ptr = dataHeadNode->head;
    if(ptr == NULL)
        return;

    PktData pktData;
    FILE *stream = fopen(fileName, "w+");
    //fprintf(stream, "AccX\t\tAccY\t\tAccZ\t\tGyroX\t\tGyroY\t\tGyroZ\t\tMagX\t\tMagY\t\tMagZ\t\tNumber\tTimeStamp\r\n");
    int i = 0;
    while (ptr != NULL) {
        pktData = ptr->packetData;
        i++;
        fprintf(stream, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%d\n",
        pktData.accX, pktData.accY, pktData.accZ,
        pktData.gyroX, pktData.gyroY, pktData.gyroZ,pktData.magX,pktData.magY,pktData.magZ,i
        );
        ptr = ptr->next;
    }
    fclose(stream);
}

void write_distance_to_file(char * fileName, int num, double d, bool is)
{
    FILE *stream = fopen(fileName, "a+");
    fprintf(stream, "%d\t%lf\t%d\n", num,d > DBL_MAX - 1 ? 6000 : d,is == true ? d : 0);
    fclose(stream);
}

/**
*save a new gesture into list.txt
*/
void insert_new_custom_gesture_item(CustomGestureItem item)
{
    FILE *stream = fopen("./custom_gesture/list.txt", "a+");
    fprintf(stream, "%d\t%s\t%d\n", item.gestureFunction, item.gestureName, item.magNum);
    printf("insert list done!\n");
    fclose(stream);
}

/**
*load all gesture from list.txt
*/
void load_custom_gesture_list(CustomGestureList *cList)
{
    FILE * fp;
    /* open */
    if ((fp = fopen("./custom_gesture/list.txt","r")) == NULL) {
        printf("Can't open ./custom_gesture/list.txt\n");
        exit(1);
    }

    bool isFirst = true;
    int type;

    int i = 0;
    CustomGestureItem *p;
    char tmpName[60];
    int magNum;
    while(EOF != fscanf(fp, "%d\t%s\t%d",&type,tmpName,&magNum))
    {
        char *cgName = (char*)malloc(sizeof(char) * 61);
        sprintf(cgName, "%s", tmpName);
        CustomGestureItem *item = (CustomGestureItem*) malloc(sizeof(CustomGestureItem));
        item->gestureFunction = type;
        item->gestureName = cgName;
        item->magNum = magNum;

        if(isFirst)
        {
            isFirst = false;
            cList->head = item;
        }
        else
        {
            p->next = item;
        }

        p = item;
        i++;
    }
    p->next = NULL;

    /* close */
    fclose(fp);

    cList->length = i;
}

/**
*save user gesture parameters into txt file
*/
void save_user_template_parameter(double threshold, int timeSpan, char * name)
{
    FILE *stream = fopen(name, "a+");
    fprintf(stream, "%lf\t%d\n", threshold, timeSpan);
    fclose(stream);
}

/**
*save a normalized mag template into txt file
*/
void save_mag_template(char * fileName, AverageList *userMagData)
{
    FILE *stream = fopen(fileName, "a+");
    while(userMagData != NULL)
    {
        fprintf(stream, "%lf\t%lf\t%lf\n", userMagData->head->packetData.magX, userMagData->head->packetData.magY, userMagData->head->packetData.magZ);
        userMagData = userMagData->pre;
    }

    fclose(stream);
}

/**
*save a path just for debug
*/
void save_path_template(char *pathFileName,WarpingPathTypeItem * pathList1)
{
    FILE *stream = fopen(pathFileName, "a+");
    while(pathList1 != NULL)
    {
        fprintf(stream, "%d\t%d\n", pathList1->position,pathList1->y);
        pathList1 = pathList1->pre;
    }
    fclose(stream);
}
