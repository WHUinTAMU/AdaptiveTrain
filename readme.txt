---------------------source files explanation--------------------------
main : the main body of the program.

PktParaser : 

SerialPort : 

SPRING : the spring dtw algorithm, and mag template normalization

TargetRecognition : convert mag data to angle value and compute the target in the old version

cjson : function of parasing and creating json

DataCalibrator : calibrate mag data in the old version

DataNode : some data structures for raw data

FileUtil : some functions of file operation

GSLMatrixs : 

LampCmd : 

LampController : create lamp control command.



------------------other files explanation---------------------------
activity_model : physical activities templates in old version

custom_gesture : files about custom gestures, including 
		list.txt : the list of all custom gestures, including the function number, name, number of mag templates
		xxx_template.txt : the template of the custom gesture
		xxx_parameter.txt : the threshold and time limitation of the gesture
		xxx_magtemplate_x.txt : the mag template of a gesture

gesture_model : gestures templates in old version and click action template


-----------------instructions of operation--------------------------
1. run the program, push the button in the Hue Lamp bridge if see the tips in the console
2. select the item in the main menu
3. "2" is to create new gesture. First, input the name, and select the function, then shack the MotionNet to start the gesture, 
   and shack again to stop the gesture. Then, repeate the gesture until the tip show in the console. Last, input the number of 
   the targets you need, and do the gesture to these targets.
4. "3" is to load the gestures already saved. Input the number in front of each gesture, input "1" to back to main menu.
5. after load gestures, in the main menu, input "1" to start control lamps.  






