PROJECT 3- Phase 4 :- V-rep Implementation of A*
-Preyash Parikh
- Ajinkya Parwekar

Python libraries to be imported :-

                      -math
	-sys
	-matplotlib.pyplot
	-time

Important Points :-
	
	- Make sure "sim.py" ,"simpleTest.py" and "simConst.py" are present in the same folder as the execution python file(vrep_main_python.py). These files can be found at -  programming\remoteApiBindings\python\python. If its 	   	  vrep.py and vrepConst.py file, please replace sim with vrep wherever necessary in the main file.
	
	- File "remoteApi.dll" is also to be placed in the same folder as above. For windows .dll file is needed, for other OS compatible files needed to be placed in same folder.
	
	- V-REP (VREP_map) scene is also to be placed in the same folder as above.
	
	-To include turtlebot model in your V-REP, add "Turtlebot2.ttm" in installation folder of V-REP at  "models\robots\mobile".

	-The code is divided in obstacle.py and vrep_main_python.Both the files  has to be placed in same folder.

	-From all the python files, for output run obstacle.py and vrep_main_python.py file.

	- Add "simRemoteApi.start(19999)" at start in child script in V-REP.


Parameters defined :-

	- Clearance = 2
	- RPM 1 = 	50	
	- RPM 2 = 100
	- Length between wheels :- 354 mm
	- Clearance for the obstacle space using half planes((diameter of robot/2)+clearance) = 2.177
 
- Play the VREP environment and then run the code.
- The input for the videos is Zero radians.
-(0,0) is at centre of VREP_map
- In vrep_main_python.py file copy the "out" from console and paste at line number 344. This is the output.
- There are two videos :-	
	1) For Start and End point (-4,-3) and (0,-3) respectively, Video name is - 1.avi
	2) For Start and End point  (-4,-3) and (4,3)respectively, Video name is - 2.avi
  
