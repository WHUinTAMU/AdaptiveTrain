#ifndef LAMPCONTROLLER_H_INCLUDED
#define LAMPCONTROLLER_H_INCLUDED
#include <stdbool.h>
#include "cJSON.h"
#include "LampCmd.h"
#include <stdio.h>

//control more detailed type
#define ON_OFF_TYPE 0
#define BRI_UP_TYPE 1
#define BRI_DOWN_TYPE 2
#define HUE_UP_TYPE 3
#define HUE_DOWN_TYPE 4

//control type
#define ON_TYPE 0
#define BRI_TYPE 1
#define HUE_TYPE 2

#define ON_STRING "on"
#define BRI_STRING "bri"
#define HUE_STRING "hue"

//the value that an action can change
#define BRI_VALUE_UP 50
#define BRI_VALUE_DOWN -50
#define HUE_VALUE_UP 5000
#define HUE_VALUE_DOWN -5000

bool onValue[4];
int briValue[4];
int hueValue[4];

/**
*TASK:based on the gesture, create the command and control the lamps
*stateType:ON_OFF_TYPE, BRI_UP_TYPE, BRI_DOWN_TYPE, HUE_UP_TYPE, HUE_DOWN_TYPE
*target:the number of  lamp, start from 0
*/
bool createCommand(int stateType, int target);

#endif // LAMPCONTROLLER_H_INCLUDED
