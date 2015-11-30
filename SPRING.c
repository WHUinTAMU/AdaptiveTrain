#include "SPRING.h"

/**
*TASK:the process struct of one kind of gesture
*grp:the process struct of the specific type of gesture
*xt:the current data inputed
*/
void update_array(GRProcess *grp, PktData xt, int position)
{
    char from[2];
    grp->distanceArray[0] = 0;
    grp->startArray[0] = position;
    grp->timeArray[0] = xt.timeStamp;
    DataNode *p = grp->originalGesture.head->head;
    int m = grp->originalGesture.m;

    int i = 0;
    /*for(i = 0; i <= m;i++)
    {
        printf("%lf  ",grp->distanceArrayLast[i]);
    }*/

    for(i = 1; i < m + 1; i++)
    {
        int startTmp;
        long int timeTmp;
        double distanceTmp = pow((xt.accX - p->packetData.accX), 2)
        + pow((xt.accY - p->packetData.accY), 2) + pow((xt.accZ - p->packetData.accZ), 2)
        + pow((xt.gyroX - p->packetData.gyroX), 2) + pow((xt.gyroY - p->packetData.gyroY), 2)
        + pow((xt.gyroZ - p->packetData.gyroZ), 2);

        //printf("d[%d - 1] = %lf:::d'[%d] = %lf:::d'[%d - 1] = %lf\n",i,grp->distanceArray[i - 1],i,grp->distanceArrayLast[i],i,grp->distanceArrayLast[i - 1]);

        if(grp->distanceArray[i - 1] <= grp->distanceArrayLast[i])
        {
            if(grp->distanceArray[i - 1] <= grp->distanceArrayLast[i - 1])
            {
                distanceTmp += grp->distanceArray[i - 1];
                startTmp = grp->startArray[i - 1];
                timeTmp = grp->timeArray[i - 1];
                from[0] = '<';
                from[1] = '-';
            }
            else
            {
                distanceTmp += grp->distanceArrayLast[i - 1];
                startTmp = grp->startArrayLast[i - 1];
                timeTmp = grp->timeArrayLast[i - 1];
                from[0] = '^';
                from[1] = '\\';
            }
        }
        else
        {
            if(grp->distanceArrayLast[i] <= grp->distanceArrayLast[i - 1])
            {
                distanceTmp += grp->distanceArrayLast[i];
                startTmp = grp->startArrayLast[i];
                timeTmp = grp->timeArrayLast[i];
                from[0] = '^';
                from[1] = '|';
            }
            else
            {
                distanceTmp += grp->distanceArrayLast[i - 1];
                startTmp = grp->startArrayLast[i - 1];
                timeTmp = grp->timeArrayLast[i - 1];
                from[0] = '^';
                from[1] = '\\';
            }
        }
        grp->distanceArray[i] = distanceTmp;
        grp->startArray[i] = startTmp;
        grp->timeArray[i] = timeTmp;
        p = p->next;
    }
}

/**
*TASK:the main part of the DTW algorithm
*grProcess:the process struct of the specific type of gesture
*xt:the type recognized
*/
int SPRING(PktData xt, GRProcess *grProcess, int position, SqQueue* queue, bool isSkip, bool isWriteDistance, bool isPrint, int target)
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

    update_array(grProcess, xt, position);

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

            if(grProcess->startArray[i] <= *te)
            {
                is_si_largerthan_te = false;
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
                        case CLICK_TYPE:printf("\n\n!!!!!!!!\nsuccess!\nCLICK!!!\ndmin=%f\nts=%d\nte=%d\nt=%d\ntime span=%d\n!!!!!!!!\n\n",
                                 *dmin,*ts,*te,position,*timee - *times);is_gesture = CLICK_TYPE;break;
                        case CUSTOM_TYPE:createCommand(grProcess->functionNum,target);printf("\n\n!!!!!!!!\nsuccess!\n%s!!!\nfunction is %d!!!!!\ndmin=%f\nts=%d\nte=%d\nt=%d\ntime span=%d\n!!!!!!!!\n\n"
                                ,grProcess->name,grProcess->functionNum
                                 ,*dmin,*ts,*te,position,*timee - *times);is_gesture = CUSTOM_TYPE;break;

                    }
                }

            }

        }
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
    printf("!!!!!!!!!!!!!!!!!!!!!%f!!!!!!!!!!!!!!!!!!!!\n",distanceMetrix[inputLength - 1][templateLength - 1]);
    return distanceMetrix[inputLength - 1][templateLength - 1];
}
