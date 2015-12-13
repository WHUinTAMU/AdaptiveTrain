#include "LampController.h"

//the initial states of these lamp
extern bool onValue[4] = {true,true,true,true};
extern int briValue[4] = {254,254,254,254};
extern int hueValue[4] = {31767,31767,31767,31767};

/**
*TASK:based on the gesture, create the command and control the lamps
*stateType:ON_OFF_TYPE, BRI_UP_TYPE, BRI_DOWN_TYPE, HUE_UP_TYPE, HUE_DOWN_TYPE
*target:the number of  lamp, start from 0
*/
bool createCommand(int stateType, int target)
{
    cJSON * pJsonRoot = NULL;
    pJsonRoot = cJSON_CreateObject();
    if(NULL == pJsonRoot)
    {
        //error happend here
        printf("error when create json");
        return false;
    }

    switch(stateType)
    {
    case ON_TYPE:
    {
        onValue[target - 1] = (onValue[target - 1] == true ? false : true);
        cJSON_AddBoolToObject(pJsonRoot, ON_STRING, onValue[target - 1]);
        break;
    }
    case BRI_UP_TYPE:
    {
        int value = (briValue[target - 1] + BRI_VALUE_UP) > 254 ? 254 : (briValue[target - 1]
                    + BRI_VALUE_UP < 0 ? 0 : briValue[target - 1] + BRI_VALUE_UP);
        briValue[target - 1] = value;
        cJSON_AddNumberToObject(pJsonRoot, BRI_STRING, value);
        break;
    }
    case BRI_DOWN_TYPE:
    {
        int value = (briValue[target - 1] + BRI_VALUE_DOWN) > 254 ? 254 : (briValue[target - 1]
                    + BRI_VALUE_DOWN < 0 ? 0 : briValue[target - 1] + BRI_VALUE_DOWN);
        briValue[target - 1] = value;
        cJSON_AddNumberToObject(pJsonRoot, BRI_STRING, value);
        break;
    }
    case HUE_UP_TYPE:
    {
        int value = (hueValue[target - 1] + HUE_VALUE_UP);
        value = value < 0 ? 65535 + value : value % 65535;
        hueValue[target - 1] = value;
        cJSON_AddNumberToObject(pJsonRoot, HUE_STRING, value);
        break;
    }
    case HUE_DOWN_TYPE:
    {
        int value = (hueValue[target - 1] + HUE_VALUE_DOWN);
        value = value < 0 ? 65535 + value : value % 65535;
        hueValue[target - 1] = value;
        cJSON_AddNumberToObject(pJsonRoot, HUE_STRING, value);
        break;
    }
    }

    printf("%s",cJSON_Print(pJsonRoot));
    setLightState(target, cJSON_Print(pJsonRoot));
    cJSON_Delete(pJsonRoot);
}
