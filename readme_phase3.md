1. libraries used:
	import sys
	import cv2
	import time
	import argparse
	from heapq import heappush, heappop
	import math
	import matplotlib.pyplot as plt

2. The second line in the code files "sys.path.remove(sys.path[1])" is written because if the code is run in ubuntu, it may cause problems for cv2 library.
If not needed, it can be commented out.

3. The animation video is made for coordinates as: (-4, -3) [bottom left corner of map], 45 [orientation], (4, 3) [top right corner] of the map, clearance = 1, rpm1 = 50, rpm2 = 100.

4. Actual animation of the exploration wont be displayed because it takes much time to produce the animation and will slow down the process.

5. The final animation video is provided in the folder along with code.

6. The code is divided in two parts: main.py and obstacle.py
   Both files are to be placed in the same folder in order to run the code properly.
