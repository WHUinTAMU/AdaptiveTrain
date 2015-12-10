#include "SPRING.h"

/**
*TASK:the process struct of one kind of gesture
*grp:the process struct of the specific type of gesture
*xt:the current data inputed
*/
void update_array(GRProcess *grp, PktData xt, int position, bool usePath)
{
    int m = grp->originalGesture.m;
    WarpingPathItem * wpItemListTmp = (WarpingPathItem *) malloc(sizeof(WarpingPathItem) * (m + 1));
    grp->distanceArray[0] = 0;
    grp->startArray[0] = position;
    grp->timeArray[0] = xt.timeStamp;
    DataNode *p = grp->originalGesture.head->head;

    int i = 0;
    /*for(i = 0; i <= m;i++)
    {
        printf("%lf  ",grp->distanceArrayLast[i]);
    }*/
    //printf("@@@@@@@@@%d\n",m);
    for(i = 1; i < m + 1; i++)
    {
        //printf("!!!%d\n",i);
        int startTmp;
        long int timeTmp;
        double distanceTmp = pow((xt.accX - p->packetData.accX), 2)
        + pow((xt.accY - p->packetData.accY), 2) + pow((xt.accZ - p->packetData.accZ), 2)
        + pow((xt.gyroX - p->packetData.gyroX), 2) + pow((xt.gyroY - p->packetData.gyroY), 2)
        + pow((xt.gyroZ - p->packetData.gyroZ), 2);

        //printf("d[%d - 1] = %lf:::d'[%d] = %lf:::d'[%d - 1] = %lf\n",i,grp->distanceArray[i - 1],i,grp->distanceArrayLast[i],i,grp->distanceArrayLast[i - 1]);
        WarpingPathItem *wpItem;
        if(usePath)
        {
            //printf("1\n");
            wpItem = (WarpingPathItem*) malloc(sizeof(WarpingPathItem));
        }

        if(grp->distanceArray[i - 1] <= grp->distanceArrayLast[i])
        {
            if(grp->distanceArray[i - 1] <= grp->distanceArrayLast[i - 1])
            {
                distanceTmp += grp->distanceArray[i - 1];
                startTmp = grp->startArray[i - 1];
                timeTmp = grp->timeArray[i - 1];
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
        //printf("2\n");
        if(usePath)
        {
            wpItemListTmp[i] = *wpItem;
        }
        //printf("3\n");

        p = p->next;
    }
    grp->warpingPathArray = wpItemListTmp;
}

/**
*TASK:the main part of the DTW algorithm
*grProcess:the process struct of the specific type of gesture
*xt:the type recognized
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

        //whether dm > dmin
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
            //printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%d\n",timeGap);
            if(timeGap >= grProcess->timeLimit)
            {
                //is_gesture = true;
                int t = grProcess->type;

                //report the right optimal subsequence
/*                 case POINT_TYPE:createCommand(ON_TYPE,-1,target);printf("\n\n!!!!!!!!\nsuccess!\npoint!!!\ndmin=%f\nts=%d\nte=%d\nt=%d\ntime span=%d\n!!!!!!!!\n\n",
 *                                   *dmin,*ts,*te,position,*timee - *times);is_gesture = POINT_TYPE;break;
 *                     case ROTATE_RIGHT_TYPE:createCommand(BRI_TYPE,BRI_VALUE_UP,target);printf("\n\n!!!!!!!!\nsuccess!\nrotate right!!!\ndmin=%f\nts=%d\nte=%d\nt=%d\ntime span=%d\n!!!!!!!!\n\n",
 *                                   *dmin,*ts,*te,position,*timee - *times);is_gesture = ROTATE_RIGHT_TYPE;break;
 *                     case ROTATE_LEFT_TYPE:createCommand(BRI_TYPE,BRI_VALUE_DOWN,target);printf("\n\n!!!!!!!!\nsuccess!\nrotate left!!!\ndmin=%f\nts=%d\nte=%d\nt=%d\ntime span=%d\n!!!!!!!!\n\n",
 *                                   *dmin,*ts,*te,position,*timee - *times);is_gesture = ROTATE_LEFT_TYPE;break;
 *
 */
                if(isSkip != true)
                {
                    switch(t)
                    {
                        case POINT_TYPE:printf("\n\n!!!!!!!!\nsuccess!\npoint!!!\ndmin=%f\nts=%d\nte=%d\nt=%d\ntime span=%d\n!!!!!!!!\n\n",
                                  *dmin,*ts,*te,position,*timee - *times);is_gesture = POINT_TYPE;break;
                        case SLIDE_OVER_TYPE:printf("\n\n!!!!!!!!\nsuccess!\nslide over!!!\ndmin=%f\nts=%d\nte=%d\nt=%d\ntime span=%d\n!!!!!!!!\n\n",
                                  *dmin,*ts,*te,position,*timee - *times);is_gesture = SLIDE_OVER_TYPE;break;
                        case STAND_UP_TYPE:printf("\n\n!!!!!!!!\nsuccess!\nstand up!!!\ndmin=%f\nts=%d\nte=%d\nt=%d\ntime span=%d\n!!!!!!!!\n\n",
                                  *dmin,*ts,*te,position,*timee - *times);is_gesture = STAND_UP_TYPE;break;
                        case SIT_DOWN_TYPE:printf("\n\n!!!!!!!!\nsuccess!\nsit down!!!\ndmin=%f\nts=%d\nte=%d\nt=%d\ntime span=%d\n!!!!!!!!\n\n",
                                  *dmin,*ts,*te,position,*timee - *times);is_gesture = SIT_DOWN_TYPE;break;
                        case TARGET_TYPE:/*printf("\n\n!!!!!!!!\nsuccess!\ntarget!!!\ndmin=%f\nts=%d\nte=%d\nt=%d\ntime span=%d\n!!!!!!!!\n\n",
                                  *dmin,*ts,*te,position,*timee - *times);*/is_gesture = TARGET_TYPE;break;
                        case WALK_TYPE:printf("\n\n!!!!!!!!\nsuccess!\nwalk!!!\ndmin=%f\nts=%d\nte=%d\nt=%d\ntime span=%d\n!!!!!!!!\n\n",
                                  *dmin,*ts,*te,position,*timee - *times);is_gesture = WALK_TYPE;break;
                        case ROTATE_RIGHT_HALF_TYPE:printf("\n\n!!!!!!!!\nsuccess!\nrotate right half!!!\ndegree=%f\n\ndmin=%f\nts=%d\nte=%d\nt=%d\ntime span=%d\n!!!!!!!!\n\n",
                                  getDegreeFromGyro(*ts,*te,queue),*dmin,*ts,*te,position,*timee - *times);is_gesture = ROTATE_RIGHT_HALF_TYPE;break;
                        case ROTATE_RIGHT_FULL_TYPE:printf("\n\n!!!!!!!!\nsuccess!\nrotate right full!!!\ndegree=%f\n\ndmin=%f\nts=%d\nte=%d\nt=%d\ntime span=%d\n!!!!!!!!\n\n",
                                  getDegreeFromGyro(*ts,*te,queue),*dmin,*ts,*te,position,*timee - *times);is_gesture = ROTATE_RIGHT_FULL_TYPE;break;
                        case ROTATE_LEFT_HALF_TYPE:printf("\n\n!!!!!!!!\nsuccess!\nrotate left half!!!\ndegree=%f\n\ndmin=%f\nts=%d\nte=%d\nt=%d\ntime span=%d\n!!!!!!!!\n\n",
                                  getDegreeFromGyro(*ts,*te,queue),*dmin,*ts,*te,position,*timee - *times);is_gesture = ROTATE_LEFT_HALF_TYPE;break;
                        case ROTATE_LEFT_FULL_TYPE:printf("\n\n!!!!!!!!\nsuccess!\nrotate left full!!!\ndegree=%f\n\ndmin=%f\nts=%d\nte=%d\nt=%d\ntime span=%d\n!!!!!!!!\n\n",
                                  getDegreeFromGyro(*ts,*te,queue),*dmin,*ts,*te,position,*timee - *times);is_gesture = ROTATE_LEFT_FULL_TYPE;break;
                        case CLICK_TYPE:printf("\n\n!!!!!!!!\nsuccess!\nCLICK!!!\n"/*dmin=%f\nts=%d\nte=%d\nt=%d\ntime span=%d\n!!!!!!!!\n\n",
                                 *dmin,*ts,*te,position,*timee - *times*/);is_gesture = CLICK_TYPE;break;
                        case CUSTOM_TYPE:printf("\n\n!!!!!!!!\nsuccess!\n%s!!!\nfunction is %d!!!!!\ndmin=%f\nts=%d\nte=%d\nt=%d\ntime span=%d\n!!!!!!!!\n\n"
                                ,grProcess->name,grProcess->functionNum
                                 ,*dmin,*ts,*te,position,*timee - *times);is_gesture = CUSTOM_TYPE;break;

                    }
                    //printf("end - start = %d\n",*te - *ts);

                    if(usePath)
                    {
                        //WarpingPathTypeItem *pathList;

                        WarpingPathList *tmpWP = grProcess->warpingPathMetrixHead.tail;
                        //printf("start position of metrix:%d!!!!!!!!!!!end:%d\n",grProcess->warpingPathMetrixHead.headNum,grProcess->warpingPathMetrixHead.tail->position);
                        //printf("head !!!!!!x = %d;;;y = %d;;;fx = %d;;;fy = %d\n",grProcess->warpingPathMetrixHead.head->itemArray[1].x,grProcess->warpingPathMetrixHead.head->itemArray[1].y,grProcess->warpingPathMetrixHead.head->itemArray[1].fx,grProcess->warpingPathMetrixHead.head->itemArray[1].fy);
                        //printf("tail!!!!x = %d;;;y = %d;;;fx = %d;;;fy = %d\n",grProcess->warpingPathMetrixHead.tail->itemArray[m].x,grProcess->warpingPathMetrixHead.tail->itemArray[m].y,grProcess->warpingPathMetrixHead.tail->itemArray[m].fx,grProcess->warpingPathMetrixHead.tail->itemArray[1].fy);
                        bool isFirstType = true;
                        int itemNum = m;
                        WarpingPathItem *tmpItem = &(tmpWP->itemArray[itemNum]);

                        while(tmpItem->y > 1)
                        {

                            //printf("y = %d\ntype = %d\nposition = %d\n",tmpItem->y,tmpItem->path,tmpWP->position);
                            WarpingPathTypeItem *type = (WarpingPathTypeItem*) malloc(sizeof(WarpingPathTypeItem));
                            switch(tmpItem->path)
                            {
                                case PATH_LEFT:{

                                    type->type = PATH_LEFT;
                                    type->position = tmpItem->x;
                                    type->y = tmpItem->y;
                                    tmpWP = tmpWP->pre;
                                    tmpItem = &(tmpWP->itemArray[itemNum]);
                                    break;
                                }
                                case PATH_DOWN:{
                                    type->type = PATH_DOWN;
                                    type->position = tmpItem->x;
                                    type->y = tmpItem->y;
                                    tmpItem = &(tmpWP->itemArray[--itemNum]);
                                    break;
                                }
                                case PATH_CORNER:{
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
//
//                        //printf("path: ");
//                        char pathFileName[100];
//                        sprintf(pathFileName, "./custom_gesture/%s_pathTemplate.txt", grProcess->name);
//                        WarpingPathTypeItem *pathList1 = *pathList;
//
//                        //printf("save %s 's %d th mag template\n",magTempFileName, magNumTmp);
//                        save_path_template(pathFileName,pathList1);
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
            //printf("1\n");
            WarpingPathList *tmpWPList = grProcess->warpingPathMetrixHead.head;
            //printf("tmpWPList.itemArray[1].x = %d\n",tmpWPList->itemArray[1].x);
            WarpingPathList *tmpFreeWP;
            //printf("3\n");
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
        //printf("startArray[m] = %d\n",grProcess->startArray[m]);
        //printf("headNum = %d;;;;head position = %d;;;head itemArray.x = %d",grProcess->warpingPathMetrixHead.headNum,grProcess->warpingPathMetrixHead.head->position,grProcess->warpingPathMetrixHead.head->itemArray[1].x);
        //printf("length:%d;;;;;headNum = %d;;;;;;;tailNum = %d\n",grProcess->warpingPathMetrixHead.length,grProcess->warpingPathMetrixHead.head->position,grProcess->warpingPathMetrixHead.tail->position);
        //printf("head !!!!!!x = %d;;;y = %d;;;fx = %d;;;fy = %d\n\n",grProcess->warpingPathMetrixHead.head->itemArray[1].x,grProcess->warpingPathMetrixHead.head->itemArray[1].y,grProcess->warpingPathMetrixHead.head->itemArray[1].fx,grProcess->warpingPathMetrixHead.head->itemArray[1].fy);
        //printf("tail!!!!x = %d;;;y = %d;;;fx = %d;;;fy = %d\n\n",grProcess->warpingPathMetrixHead.tail->itemArray[m].x,grProcess->warpingPathMetrixHead.tail->itemArray[m].y,grProcess->warpingPathMetrixHead.tail->itemArray[m].fx,grProcess->warpingPathMetrixHead.tail->itemArray[1].fy);

    }

    if(isSkip || is_gesture != NONE_TYPE)
    {
        //reinitialize the dmin,d
        *dmin = DBL_MAX;

        int i = 0;
        for(i = 1; i <= m; i++)
        {
            //if(compare_two_position(queue, *te, grProcess->startArray[i]))
            //{
                grProcess->distanceArray[i] = DBL_MAX;
            //}
        }
    }

    //check whether the current subsequence can be determined as a temporary optimal subsequence
    //printf("distance = %f ::::: dmin = %f\n",grProcess->distanceArray[m],*dmin);
    if(grProcess->distanceArray[m] <= threshold && grProcess->distanceArray[m] < *dmin)
    {
        *dmin = grProcess->distanceArray[m];
        *ts = grProcess->startArray[m];
        *te = position;
        *times = grProcess->timeArray[m];
        *timee = xt.timeStamp;

        //printf("dmin=%lf:::te=%d:::ts=%d:::timee=%d:::times=%d:::time=%d\n",*dmin,*te,*ts,*timee,*times,*timee-*times);
    }
    if(isWriteDistance)
    {
        write_distance_to_file("./distance.txt", xt.pktNumber, grProcess->distanceArray[m], is_gesture == CUSTOM_TYPE);
    }
    if(isPrint)
        printf("%d::distance = %lf::start = %d::start = %d::end = %d\n", xt.pktNumber, grProcess->distanceArray[m], grProcess->startArray[m],*ts,*te);
        //printf("%d::distance = %lf::start = %d::time span = %d\n", xt.pktNumber, grProcess->distanceArray[m], grProcess->startArray[m],*timee - *times);

 //printf("start = %d::end = %d\n", *timee , *times);

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
    //printf("!!!!!!!!!!!!!!!!!!!!!%f!!!!!!!!!!!!!!!!!!!!\n",distanceMetrix[inputLength - 1][templateLength - 1]);
    return distanceMetrix[inputLength - 1][templateLength - 1];
}


DataHeadNode *transferSqQueueToDhn(SqQueue *queue, int start, int end)
{
    bool isFirst = true;
    DataNode *tmp = NULL;
    DataNode *head = NULL;
    int length = end - start >= 0 ? end - start  + 1 : MAX_SIZE + end - start + 1;
    int i = start;
    int j = 0;
    for(j = 0; j < length;j++)
    {
        PktData *pd = (PktData*) malloc(sizeof(PktData));
//        pd->accX = queue->accXData[i];
//        pd->accY = queue->accYData[i];
//        pd->accZ = queue->accZData[i];
//        pd->gyroX = queue->gyroXData[i];
//        pd->gyroY = queue->gyroYData[i];
//        pd->gyroZ = queue->gyroZData[i];
        pd->magX = queue->magXData[i];
        pd->magY = queue->magYData[i];
        pd->magZ = queue->magZData[i];

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
            tmp->next = NULL;
            i = (i + 1) % (MAX_SIZE - 1);
    }
    tmp->next = NULL;

    DataHeadNode *dhn = (DataHeadNode*) malloc(sizeof(DataHeadNode));
    dhn->head = head;

    return dhn;


}

//Determine whether the stack is empty
bool empty(PSTACK pS)
{
    if(pS->pTop == NULL)//判断栈是否为空
        return true;
    else
        return false;

}

// Define the Push of STACK
void push(PSTACK pS, DataNode *val)
{
    //printf("push!!!!!");
    PNODE pNew = (PNODE)malloc(sizeof(NODE));    //    定义一个新节点，并分配内存空间
    if (NULL == pNew)
    {
        return ;
    }

    pNew->data = val;
    pNew->pNext = pS->pTop;   //压栈时让栈顶指向新生成的节点
    pS->pTop = pNew;
    //printf("push top !!!! %lf\t%lf\t%lf",pS->pTop->data->packetData.magX,pS->pTop->data->packetData.magY,pS->pTop->data->packetData.magZ);

    return;
}


//Define the Pop of STACK
bool pop(PSTACK pS, DataNode **pVal)
{
    //printf("pop!!!!!!");
    //printf("top !!!! %lf\t%lf\t%lf\n",pS->pTop->data->packetData.magX,pS->pTop->data->packetData.magY,pS->pTop->data->packetData.magZ);

    if(empty(pS))//由于引进来时pS就是地址
    {
        return false;
    }
    else
    {
        (*pVal) = pS->pTop->data;
        pS->pTop = pS->pTop->pNext;

        //printf("%lf\t%lf\t%lf",(*pVal)->packetData.magX,(*pVal)->packetData.magY,(*pVal)->packetData.magZ);
        return true;
    }
}

AverageList *Normalization(WarpingPathTypeItem *wpTypeItemTail, SqQueue *queue, int s, int e)
{
    STACK *StkForNormalizing = (STACK*) malloc(sizeof(STACK));
    StkForNormalizing->pBottom = NULL;
    StkForNormalizing->pTop = NULL;

    //STACK *StkForAveraging = (STACK*) malloc(sizeof(STACK));

    DataHeadNode *userdhn = transferSqQueueToDhn(queue, s, e);

    DataNode *userdn;
    userdn = userdhn->head;

    //wpTypeItemTail = (WarpingPathTypeItem*) malloc(sizeof(WarpingPathTypeItem));

    //push all userdata into the normalizing stack
    push(StkForNormalizing, userdn);

    DataNode *stackTop = userdn;
    while(userdn != NULL && wpTypeItemTail != NULL){

        //printf("%d\n",wpTypeItemTail->type);
        switch (wpTypeItemTail->type){
            case PATH_DOWN:{
                push(StkForNormalizing, stackTop);
                //printf("!!!!%d----\n",wpTypeItemTail->type);
                wpTypeItemTail = wpTypeItemTail->pre;
                break;
            }
            case PATH_CORNER:{
                userdn = userdn->next;
                stackTop = userdn;
                push(StkForNormalizing, userdn);
                //printf("@@@@@%d----\n",wpTypeItemTail->type);
                wpTypeItemTail = wpTypeItemTail->pre;
                break;
            }
            case PATH_LEFT:{
                pop(StkForNormalizing, &stackTop);

                AverageList *avrgList = (AverageList*) malloc(sizeof(AverageList));
                avrgList->pre = NULL;
                avrgList->next = NULL;
                avrgList->head = stackTop;

                AverageList *avrgListTmp = (AverageList*) malloc(sizeof(AverageList));
                userdn = userdn->next;
                avrgListTmp->head = userdn;
                avrgList->next = avrgListTmp;
                avrgListTmp->pre = avrgList;
                avrgListTmp->next = NULL;
                avrgList = avrgListTmp;
                wpTypeItemTail = wpTypeItemTail->pre;

                //Create a list for data that needs to be averaged.
                while(wpTypeItemTail != NULL && wpTypeItemTail->type == PATH_LEFT){

                    userdn = userdn->next;

                    avrgListTmp = (AverageList*) malloc(sizeof(AverageList));
                    avrgListTmp->head = userdn;
                    avrgList->next = avrgListTmp;
                    avrgListTmp->pre = avrgList;
                    avrgListTmp->next = NULL;
                    avrgList = avrgListTmp;
                    wpTypeItemTail = wpTypeItemTail->pre;
                }

                //traverse avrgList to get the number
                //average the magdata
                DataNode *averageddn = (DataNode*) malloc(sizeof(DataNode));
                int countOfAvrglist = 0;
                averageddn->packetData.magX = 0;
                averageddn->packetData.magY = 0;
                averageddn->packetData.magZ = 0;
                while(avrgList != NULL){
                    countOfAvrglist++;
                    averageddn->packetData.magX += avrgList->head->packetData.magX;
                    //printf("x = %lf\n",averageddn->packetData.magX);
                    averageddn->packetData.magY += avrgList->head->packetData.magY;
                    //printf("y = %lf\n",averageddn->packetData.magY);
                    averageddn->packetData.magZ += avrgList->head->packetData.magZ;
                    //printf("z = %lf\n",averageddn->packetData.magZ);
                    avrgList = avrgList->pre;
                }
                //printf("count = %d",countOfAvrglist);
                averageddn->packetData.magX /= countOfAvrglist;
                averageddn->packetData.magY /= countOfAvrglist;
                averageddn->packetData.magZ /= countOfAvrglist;
                push(StkForNormalizing, averageddn);
                stackTop = averageddn;
                //wpTypeItemTail = wpTypeItemTail->pre;
                break;
            }
        }

        //userdn = userdn->next;

    }

    // output the normalized user magdata backwards
    AverageList *normalizedUserData = (AverageList*) malloc(sizeof(AverageList));
    AverageList *normalizedUserDataTmp;

    normalizedUserData->pre = NULL;
    normalizedUserData->next = NULL;

    DataNode *tmpForTraverse = NULL;
    pop(StkForNormalizing, &tmpForTraverse);
    //printf("%lf\t%lf\t%lf",tmpForTraverse->packetData.magX,tmpForTraverse->packetData.magY,tmpForTraverse->packetData.magZ);
    normalizedUserData->head = tmpForTraverse;

    while(!empty(StkForNormalizing)){
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

    //get the normalized user magdata FROM BACKWARDS
    return normalizedUserData;
}


double compute_magdata_distance(AverageList *normalizedUserData, DataHeadNode *dhn)
{
    //Compute the distance between user data and template
    DataNode *dataNodeTmp = dhn->head;
    int length = dhn->length;

    AverageList *signalForUser = normalizedUserData;
    DataNode *signalForTmp = dataNodeTmp;

    double distance = 0;

    while(signalForUser != NULL){

        distance = distance + pow((signalForUser->head->packetData.magX - signalForTmp->packetData.magX), 2)
             +pow((signalForUser->head->packetData.magY - signalForTmp->packetData.magY), 2)
             +pow((signalForUser->head->packetData.magZ - signalForTmp->packetData.magZ), 2);

        signalForTmp=signalForTmp->next;
        signalForUser = signalForUser->pre;
    }
    distance = sqrt(distance);

    return distance;



}

