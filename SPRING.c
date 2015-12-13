#include "SPRING.h"

/**
*TASK:the process struct of one kind of gesture
*grp:the process struct of the specific type of gesture
*xt:the current data inputed
*position:the position of thr current data in the queue
*usePath:whether need to compute the warping path
*/
void update_array(GRProcess *grp, PktData xt, int position, bool usePath)
{
    int m = grp->originalGesture.m;
    WarpingPathItem * wpItemListTmp = (WarpingPathItem *) malloc(sizeof(WarpingPathItem) * (m + 1));
    grp->distanceArray[0] = 0;
    grp->startArray[0] = position;
    grp->timeArray[0] = xt.timeStamp;
    DataNode *p = grp->originalGesture.head->head;

    //update the distance array and start array
    int i = 0;
    for(i = 1; i < m + 1; i++)
    {
        int startTmp;
        long int timeTmp;
        double distanceTmp = pow((xt.accX - p->packetData.accX), 2)
                             + pow((xt.accY - p->packetData.accY), 2) + pow((xt.accZ - p->packetData.accZ), 2)
                             + pow((xt.gyroX - p->packetData.gyroX), 2) + pow((xt.gyroY - p->packetData.gyroY), 2)
                             + pow((xt.gyroZ - p->packetData.gyroZ), 2);

        //record the position and path of a specific place
        WarpingPathItem *wpItem;
        if(usePath)
        {
            wpItem = (WarpingPathItem*) malloc(sizeof(WarpingPathItem));
        }

        if(grp->distanceArray[i - 1] <= grp->distanceArrayLast[i])
        {
            if(grp->distanceArray[i - 1] <= grp->distanceArrayLast[i - 1])
            {
                distanceTmp += grp->distanceArray[i - 1];
                startTmp = grp->startArray[i - 1];
                timeTmp = grp->timeArray[i - 1];

                //record the position and path of a specific place
                if(usePath)
                {
                    wpItem->x = position;
                    wpItem->y = i;
                    wpItem->fx = position;
                    wpItem->fy = i - 1;
                    wpItem->path = PATH_DOWN;
                }
            }
            else
            {
                distanceTmp += grp->distanceArrayLast[i - 1];
                startTmp = grp->startArrayLast[i - 1];
                timeTmp = grp->timeArrayLast[i - 1];
                if(usePath)
                {
                    wpItem->x = position;
                    wpItem->y = i;
                    wpItem->fx = position - 1;
                    wpItem->fy = i - 1;
                    wpItem->path = PATH_CORNER;
                }
            }
        }
        else
        {
            if(grp->distanceArrayLast[i] <= grp->distanceArrayLast[i - 1])
            {
                distanceTmp += grp->distanceArrayLast[i];
                startTmp = grp->startArrayLast[i];
                timeTmp = grp->timeArrayLast[i];
                if(usePath)
                {
                    wpItem->x = position;
                    wpItem->y = i;
                    wpItem->fx = position - 1;
                    wpItem->fy = i;
                    wpItem->path = PATH_LEFT;
                }
            }
            else
            {
                distanceTmp += grp->distanceArrayLast[i - 1];
                startTmp = grp->startArrayLast[i - 1];
                timeTmp = grp->timeArrayLast[i - 1];
                if(usePath)
                {
                    wpItem->x = position;
                    wpItem->y = i;
                    wpItem->fx = position - 1;
                    wpItem->fy = i - 1;
                    wpItem->path = PATH_CORNER;
                }
            }
        }
        grp->distanceArray[i] = distanceTmp;
        grp->startArray[i] = startTmp;
        grp->timeArray[i] = timeTmp;

        if(usePath)
        {
            wpItemListTmp[i] = *wpItem;
        }

        p = p->next;
    }
    grp->warpingPathArray = wpItemListTmp;
}

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
int SPRING(PktData xt, GRProcess *grProcess, int position, SqQueue* queue, bool isSkip, bool isWriteDistance, bool isPrint, bool usePath, WarpingPathTypeItem **pathList)
{
    int is_gesture = NONE_TYPE;

    int m = grProcess->originalGesture.m;
    double *dmin = &(grProcess->dmin);
    int *ts = &(grProcess->ts);
    int *te = &(grProcess->te);
    long int *times = &(grProcess->times);
    long int *timee = &(grProcess->timee);
    DataHeadNode *originalGestureList = grProcess->originalGesture.head;
    double threshold = grProcess->threshold;

    update_array(grProcess, xt, position, usePath);

    //check whether the temporary optimal subsequence is a right one
    if(*dmin <= threshold)
    {
        bool is_di_largerthan_dmin = true;
        bool is_si_largerthan_te = true;

        int i = 0;
        for(i = m; i <= m; i++)
        {
            if(grProcess->distanceArray[i] < *dmin)
            {
                is_di_largerthan_dmin = false;
            }

            if(is_di_largerthan_dmin == false /*&& is_si_largerthan_te == false*/)
            {
                break;
            }
        }

        if(is_di_largerthan_dmin == true /*|| is_si_largerthan_te == true*/)
        {
            //check the time span of the temporary optimal subsequence
            int timeGap = *timee - *times;
            if(timeGap >= grProcess->timeLimit)
            {
                //is_gesture = true;
                int t = grProcess->type;

                //report the right optimal subsequence
                if(isSkip != true)
                {
                    switch(t)
                    {
//                        case POINT_TYPE:printf("\n\n!!!!!!!!\nsuccess!\npoint!!!\ndmin=%f\nts=%d\nte=%d\nt=%d\ntime span=%d\n!!!!!!!!\n\n",
//                                  *dmin,*ts,*te,position,*timee - *times);is_gesture = POINT_TYPE;break;
//                        case SLIDE_OVER_TYPE:printf("\n\n!!!!!!!!\nsuccess!\nslide over!!!\ndmin=%f\nts=%d\nte=%d\nt=%d\ntime span=%d\n!!!!!!!!\n\n",
//                                  *dmin,*ts,*te,position,*timee - *times);is_gesture = SLIDE_OVER_TYPE;break;
//                        case STAND_UP_TYPE:printf("\n\n!!!!!!!!\nsuccess!\nstand up!!!\ndmin=%f\nts=%d\nte=%d\nt=%d\ntime span=%d\n!!!!!!!!\n\n",
//                                  *dmin,*ts,*te,position,*timee - *times);is_gesture = STAND_UP_TYPE;break;
//                        case SIT_DOWN_TYPE:printf("\n\n!!!!!!!!\nsuccess!\nsit down!!!\ndmin=%f\nts=%d\nte=%d\nt=%d\ntime span=%d\n!!!!!!!!\n\n",
//                                  *dmin,*ts,*te,position,*timee - *times);is_gesture = SIT_DOWN_TYPE;break;
//                        case TARGET_TYPE:/*printf("\n\n!!!!!!!!\nsuccess!\ntarget!!!\ndmin=%f\nts=%d\nte=%d\nt=%d\ntime span=%d\n!!!!!!!!\n\n",
//                                  *dmin,*ts,*te,position,*timee - *times);*/is_gesture = TARGET_TYPE;break;
//                        case WALK_TYPE:printf("\n\n!!!!!!!!\nsuccess!\nwalk!!!\ndmin=%f\nts=%d\nte=%d\nt=%d\ntime span=%d\n!!!!!!!!\n\n",
//                                  *dmin,*ts,*te,position,*timee - *times);is_gesture = WALK_TYPE;break;
//                        case ROTATE_RIGHT_HALF_TYPE:printf("\n\n!!!!!!!!\nsuccess!\nrotate right half!!!\ndegree=%f\n\ndmin=%f\nts=%d\nte=%d\nt=%d\ntime span=%d\n!!!!!!!!\n\n",
//                                  getDegreeFromGyro(*ts,*te,queue),*dmin,*ts,*te,position,*timee - *times);is_gesture = ROTATE_RIGHT_HALF_TYPE;break;
//                        case ROTATE_RIGHT_FULL_TYPE:printf("\n\n!!!!!!!!\nsuccess!\nrotate right full!!!\ndegree=%f\n\ndmin=%f\nts=%d\nte=%d\nt=%d\ntime span=%d\n!!!!!!!!\n\n",
//                                  getDegreeFromGyro(*ts,*te,queue),*dmin,*ts,*te,position,*timee - *times);is_gesture = ROTATE_RIGHT_FULL_TYPE;break;
//                        case ROTATE_LEFT_HALF_TYPE:printf("\n\n!!!!!!!!\nsuccess!\nrotate left half!!!\ndegree=%f\n\ndmin=%f\nts=%d\nte=%d\nt=%d\ntime span=%d\n!!!!!!!!\n\n",
//                                  getDegreeFromGyro(*ts,*te,queue),*dmin,*ts,*te,position,*timee - *times);is_gesture = ROTATE_LEFT_HALF_TYPE;break;
//                        case ROTATE_LEFT_FULL_TYPE:printf("\n\n!!!!!!!!\nsuccess!\nrotate left full!!!\ndegree=%f\n\ndmin=%f\nts=%d\nte=%d\nt=%d\ntime span=%d\n!!!!!!!!\n\n",
//                                  getDegreeFromGyro(*ts,*te,queue),*dmin,*ts,*te,position,*timee - *times);is_gesture = ROTATE_LEFT_FULL_TYPE;break;
                    case CLICK_TYPE:
                        printf("\n\n!!!!!!!!\nsuccess!\nCLICK!!!\n"/*dmin=%f\nts=%d\nte=%d\nt=%d\ntime span=%d\n!!!!!!!!\n\n",
                                 *dmin,*ts,*te,position,*timee - *times*/);
                        is_gesture = CLICK_TYPE;
                        break;
                    case CUSTOM_TYPE:
                        printf("\n\n!!!!!!!!\nsuccess!\n%s!!!\nfunction is %d!!!!!\ndmin=%f\nts=%d\nte=%d\nt=%d\ntime span=%d\n!!!!!!!!\n\n"
                               ,grProcess->name,grProcess->functionNum
                               ,*dmin,*ts,*te,position,*timee - *times);
                        is_gesture = CUSTOM_TYPE;
                        break;

                    }

                    if(usePath)
                    {
                        //get the warping path from the end back to the start
                        WarpingPathList *tmpWP = grProcess->warpingPathMetrixHead.tail;
                        bool isFirstType = true;
                        int itemNum = m;
                        WarpingPathItem *tmpItem = &(tmpWP->itemArray[itemNum]);

                        while(tmpItem->y > 1)
                        {

                            WarpingPathTypeItem *type = (WarpingPathTypeItem*) malloc(sizeof(WarpingPathTypeItem));
                            switch(tmpItem->path)
                            {
                            case PATH_LEFT:
                            {

                                type->type = PATH_LEFT;
                                type->position = tmpItem->x;
                                type->y = tmpItem->y;
                                tmpWP = tmpWP->pre;
                                tmpItem = &(tmpWP->itemArray[itemNum]);
                                break;
                            }
                            case PATH_DOWN:
                            {
                                type->type = PATH_DOWN;
                                type->position = tmpItem->x;
                                type->y = tmpItem->y;
                                tmpItem = &(tmpWP->itemArray[--itemNum]);
                                break;
                            }
                            case PATH_CORNER:
                            {
                                type->type = PATH_CORNER;
                                type->position = tmpItem->x;
                                tmpWP = tmpWP->pre;
                                type->y = tmpItem->y;
                                tmpItem = &(tmpWP->itemArray[--itemNum]);
                                break;
                            }
                            }

                            if(isFirstType)
                            {
                                *pathList = type;
                                (*pathList)->next = NULL;
                                (*pathList)->pre = NULL;
                                isFirstType = false;
                            }
                            else
                            {
                                (*pathList)->next = type;
                                type->pre = (*pathList);
                                (*pathList) = type;
                                (*pathList)->next = NULL;
                            }

                        }

                    }
                }
            }
        }
    }

    if(usePath)
    {
        /** update the warping path metrix */
        WarpingPathList *newWPL = (WarpingPathList*) malloc(sizeof(WarpingPathList));
        newWPL->itemArray = grProcess->warpingPathArray;
        newWPL->lengthY = m;
        newWPL->position = position;

        grProcess->warpingPathMetrixHead.tail->next = newWPL;
        newWPL->pre = grProcess->warpingPathMetrixHead.tail;
        grProcess->warpingPathMetrixHead.tail = newWPL;
        newWPL->next = NULL;
        grProcess->warpingPathMetrixHead.length++;
        if(compare_two_position(queue, grProcess->startArray[m], grProcess->warpingPathMetrixHead.headNum))
        {
            WarpingPathList *tmpWPList = grProcess->warpingPathMetrixHead.head;
            WarpingPathList *tmpFreeWP;
            while(compare_two_position(queue, grProcess->startArray[m], tmpWPList->position))
            {
                tmpFreeWP = tmpWPList;
                tmpWPList = tmpWPList->next;
                free(tmpFreeWP);
                grProcess->warpingPathMetrixHead.length--;
            }

            grProcess->warpingPathMetrixHead.head = tmpWPList;
            grProcess->warpingPathMetrixHead.headNum = tmpWPList->position;

        }
    }

    if(isSkip || is_gesture != NONE_TYPE)
    {
        //reinitialize the dmin,d
        *dmin = DBL_MAX;

        int i = 0;
        for(i = 1; i <= m; i++)
        {
            grProcess->distanceArray[i] = DBL_MAX;
        }
    }

    //check whether the current subsequence can be determined as a temporary optimal subsequence
    if(grProcess->distanceArray[m] <= threshold && grProcess->distanceArray[m] < *dmin)
    {
        *dmin = grProcess->distanceArray[m];
        *ts = grProcess->startArray[m];
        *te = position;
        *times = grProcess->timeArray[m];
        *timee = xt.timeStamp;
    }
    if(isWriteDistance)
    {
        write_distance_to_file("./distance.txt", xt.pktNumber, grProcess->distanceArray[m], is_gesture == CUSTOM_TYPE);
    }
    if(isPrint)
        printf("%d::distance = %lf::start = %d::start = %d::end = %d\n", xt.pktNumber, grProcess->distanceArray[m], grProcess->startArray[m],*ts,*te);

    // replace the d with d', and s with s'
    double *dtmp = grProcess->distanceArray;
    grProcess->distanceArray = grProcess->distanceArrayLast;
    grProcess->distanceArrayLast = dtmp;

    int *stmp = grProcess->startArray;
    grProcess->startArray = grProcess->startArrayLast;
    grProcess->startArrayLast = stmp;

    long int *ttmp = grProcess->timeArray;
    grProcess->timeArray = grProcess->timeArrayLast;
    grProcess->timeArrayLast = ttmp;

    return is_gesture;
}

/**
*TASK:to compute the angle of rotation by the data of gyroscope
*start:the start position of the recognized subsequence in queue
*end:the end position of the recognized subsequence in queue
*queue:user data sequence
*/
double getDegreeFromGyro(int start, int end, SqQueue* queue)
{
    int i = start;
    double degree = 0;
    while(i != end + 1)
    {
        double speed = fabs(pow((pow(queue->gyroXData[i] * 2, 2) + pow(queue->gyroYData[i] * 2, 2) + pow(queue->gyroZData[i] * 2, 2)), 0.5) );
        int timeSpan = queue->timeStamp[(i + 1) % MAX_SIZE] - queue->timeStamp[i];
        degree += speed * timeSpan * 120 / 1000;

        i = (i + 1) % MAX_SIZE;
    }
    return degree ;
}

/**
*TASK:the traditional DTW distance computation
*og:the structure of a defined gesture
*head:a sequence of user data
*/
double compute_traditional_DTW(OriginalGesture *og, DataHeadNode *head)
{
    DataNode *templateP = og->head->head;
    DataNode *inputP = head->head;
    int templateLength = og->m;
    int inputLength = head->length;

    double distanceMetrix[inputLength][templateLength];

    int i,j;

    for(i = 0; i < inputLength; i++)
    {
        for(j = 0; j < templateLength; j++)
        {
            double tmpDistance = pow((templateP->packetData.accX - inputP->packetData.accX), 2)
                                 + pow((templateP->packetData.accY - inputP->packetData.accY), 2)
                                 + pow((templateP->packetData.accZ - inputP->packetData.accZ), 2)
                                 + pow((templateP->packetData.gyroX - inputP->packetData.gyroX), 2)
                                 + pow((templateP->packetData.gyroY - inputP->packetData.gyroY), 2)
                                 + pow((templateP->packetData.gyroZ - inputP->packetData.gyroZ), 2);

            if(i == 0 || j == 0)
            {
                distanceMetrix[i][j] = tmpDistance;
            }
            else
            {
                if(distanceMetrix[i - 1][j] <= distanceMetrix[i][j - 1])
                {
                    if(distanceMetrix[i - 1][j] <= distanceMetrix[i - 1][j - 1] )
                    {
                        distanceMetrix[i][j] = tmpDistance + distanceMetrix[i - 1][j];
                    }
                    else
                    {
                        distanceMetrix[i][j] = tmpDistance + distanceMetrix[i - 1][j - 1];
                    }
                }
                else
                {
                    if(distanceMetrix[i][j - 1] <= distanceMetrix[i - 1][j - 1] )
                    {
                        distanceMetrix[i][j] = tmpDistance + distanceMetrix[i][j - 1];
                    }
                    else
                    {
                        distanceMetrix[i][j] = tmpDistance + distanceMetrix[i - 1][j - 1];
                    }
                }
            }

            templateP = templateP->next;
        }
        inputP = inputP->next;
        templateP = og->head->head;
    }
    return distanceMetrix[inputLength - 1][templateLength - 1];
}

//Transfer raw user mag data in SqQueue into DataHeadNode, convenient for the normalization of user mag data.
DataHeadNode *transferSqQueueToDhn(SqQueue *queue, int start, int end)
{

    bool isFirst = true;
    DataNode *tmp = NULL;
    DataNode *head = NULL;

    //get the length of the part of the circular queue that needs traversing to be transfered into DataHeadNode
    int length = end - start >= 0 ? end - start  + 1 : MAX_SIZE + end - start + 1;
    int i = start;
    int j = 0;
    for(j = 0; j < length; j++)
    {
        //pass mag values from SqQueue to PktData
        PktData *pd = (PktData*) malloc(sizeof(PktData));

        pd->magX = queue->magXData[i];
        pd->magY = queue->magYData[i];
        pd->magZ = queue->magZData[i];

        //pass mag values from PktData to DataNode
        DataNode *dn = (DataNode*) malloc(sizeof(DataNode));
        dn->packetData = *pd;

        //determine whether this step is the first step and move the pointer
        if(isFirst)
        {
            isFirst = false;
            head = dn;
            tmp = dn;
        }
        else
        {
            tmp->next = dn;
            tmp = dn;
        }
        tmp->next = NULL;
        i = (i + 1) % (MAX_SIZE - 1);
    }
    tmp->next = NULL;

    //pass mag values from DataNode to DataHeadNode
    DataHeadNode *dhn = (DataHeadNode*) malloc(sizeof(DataHeadNode));
    dhn->head = head;

    return dhn;


}

//Determine whether the stack is empty.
bool empty(PSTACK pS)
{
    if(pS->pTop == NULL)//ÅÐ¶ÏÕ»ÊÇ·ñÎª¿Õ
        return true;
    else
        return false;

}

// Define the Push of STACK.
void push(PSTACK pS, DataNode *val)
{

    PNODE pNew = (PNODE)malloc(sizeof(NODE));
    if (NULL == pNew)
    {
        return ;
    }

    pNew->data = val;
    pNew->pNext = pS->pTop;
    pS->pTop = pNew;

    return;
}


//Define the Pop of STACK.
bool pop(PSTACK pS, DataNode **pVal)
{
    if(empty(pS))
    {
        return false;
    }
    else
    {
        (*pVal) = pS->pTop->data;
        pS->pTop = pS->pTop->pNext;

        return true;
    }
}

//Normalize the user's mag data according to the warping path extracted from accelerometer and gyroscope data.
AverageList *Normalization(WarpingPathTypeItem *wpTypeItemTail, SqQueue *queue, int s, int e)
{
    //initialize the stack that is used in the process of normalization.
    STACK *StkForNormalizing = (STACK*) malloc(sizeof(STACK));
    StkForNormalizing->pBottom = NULL;
    StkForNormalizing->pTop = NULL;

    //transfer user mag data in SqQueue into DataHeadNode.
    DataHeadNode *userdhn = transferSqQueueToDhn(queue, s, e);

    //pass mag values from DataHeadNode into DataNode, convenient for the normalization.
    DataNode *userdn;
    userdn = userdhn->head;

    //use the stack for normalizing to do the normalization.
    //push the very first set of user mag data into the normalizing stack.
    push(StkForNormalizing, userdn);


    //push the rest of user mag data into the normalizing stack.
    DataNode *stackTop = userdn;
    while(userdn != NULL && wpTypeItemTail != NULL)
    {
        switch (wpTypeItemTail->type)
        {

        //repeat pushing the current DataNode into STACK when the WarpingPathTypeItem is DOWN
        case PATH_DOWN:
        {
            push(StkForNormalizing, stackTop);
            wpTypeItemTail = wpTypeItemTail->pre;
            break;
        }

        //move to the next DataNode and push that into STACK when the WarpingPathTypeItem is CORNER.
        case PATH_CORNER:
        {
            userdn = userdn->next;
            stackTop = userdn;
            push(StkForNormalizing, userdn);
            wpTypeItemTail = wpTypeItemTail->pre;
            break;
        }

        //on the appearance of continuous LEFT, average all related mag data into a DataNode and push it into STACK.
        case PATH_LEFT:
        {

            //pop out the stackTop to participate in the averaging, as well as leaving a blank for the result of averaging.
            pop(StkForNormalizing, &stackTop);

            //initialize avrgList for the averaging of all LEFT-related mag data.
            AverageList *avrgList = (AverageList*) malloc(sizeof(AverageList));
            avrgList->pre = NULL;
            avrgList->next = NULL;
            avrgList->head = stackTop;

            //pass the first LEFT-related DataNode that needs averaging to avrgList.
            AverageList *avrgListTmp = (AverageList*) malloc(sizeof(AverageList));
            userdn = userdn->next;
            avrgListTmp->head = userdn;
            avrgList->next = avrgListTmp;
            avrgListTmp->pre = avrgList;
            avrgListTmp->next = NULL;
            avrgList = avrgListTmp;
            wpTypeItemTail = wpTypeItemTail->pre;

            //pass the rest of LEFT-related mag data to avrgList.
            while(wpTypeItemTail != NULL && wpTypeItemTail->type == PATH_LEFT)
            {

                userdn = userdn->next;

                avrgListTmp = (AverageList*) malloc(sizeof(AverageList));
                avrgListTmp->head = userdn;
                avrgList->next = avrgListTmp;
                avrgListTmp->pre = avrgList;
                avrgListTmp->next = NULL;
                avrgList = avrgListTmp;
                wpTypeItemTail = wpTypeItemTail->pre;
            }

            //traverse avrgList to get the number.
            //average the LEFT-related mag data and push into STACK in the form of DataNode.
            DataNode *averageddn = (DataNode*) malloc(sizeof(DataNode));
            int countOfAvrglist = 0;
            averageddn->packetData.magX = 0;
            averageddn->packetData.magY = 0;
            averageddn->packetData.magZ = 0;
            while(avrgList != NULL)
            {
                countOfAvrglist++;
                averageddn->packetData.magX += avrgList->head->packetData.magX;

                averageddn->packetData.magY += avrgList->head->packetData.magY;

                averageddn->packetData.magZ += avrgList->head->packetData.magZ;

                avrgList = avrgList->pre;
            }

            averageddn->packetData.magX /= countOfAvrglist;
            averageddn->packetData.magY /= countOfAvrglist;
            averageddn->packetData.magZ /= countOfAvrglist;
            push(StkForNormalizing, averageddn);
            stackTop = averageddn;

            break;
        }
        }


    }

    // output the normalized user mag data in the form of AverageList !!!BACKWARDS!!!
    AverageList *normalizedUserData = (AverageList*) malloc(sizeof(AverageList));
    AverageList *normalizedUserDataTmp;

    normalizedUserData->pre = NULL;
    normalizedUserData->next = NULL;

    DataNode *tmpForTraverse = NULL;
    pop(StkForNormalizing, &tmpForTraverse);

    normalizedUserData->head = tmpForTraverse;

    while(!empty(StkForNormalizing))
    {
        //printf("while!!!!!!!!!!!!");
        pop(StkForNormalizing, &tmpForTraverse);
        //printf("%lf\t%lf\t%lf",tmpForTraverse->packetData.magX,tmpForTraverse->packetData.magY,tmpForTraverse->packetData.magZ);
        normalizedUserDataTmp = (AverageList*) malloc(sizeof(AverageList));
        normalizedUserDataTmp->head = tmpForTraverse;
        normalizedUserData->next = normalizedUserDataTmp;
        normalizedUserDataTmp->pre =normalizedUserData;
        normalizedUserDataTmp->next = NULL;
        normalizedUserData = normalizedUserDataTmp;
    }

    //get the normalized user mag data !!!FROM BACKWARDS!!!
    return normalizedUserData;
}

//Compute the distance between normalized user mag data and the target template.
double compute_magdata_distance(AverageList *normalizedUserData, DataHeadNode *dhn)
{
    DataNode *dataNodeTmp = dhn->head;
    int length = dhn->length;
    AverageList *signalForUser = normalizedUserData;
    DataNode *signalForTmp = dataNodeTmp;

    double distance = 0;

    //distance = ((usermag.X-tmpmag.X)^2+(usermag.Y-tmpmag.Y)^2+(usermag.Z-tmpmag.Z)^2)^0.5.
    while(signalForUser != NULL)
    {

        distance = distance + pow((signalForUser->head->packetData.magX - signalForTmp->packetData.magX), 2)
                   +pow((signalForUser->head->packetData.magY - signalForTmp->packetData.magY), 2)
                   +pow((signalForUser->head->packetData.magZ - signalForTmp->packetData.magZ), 2);

        signalForTmp=signalForTmp->next;
        signalForUser = signalForUser->pre;
    }
    distance = sqrt(distance);

    return distance;
}



