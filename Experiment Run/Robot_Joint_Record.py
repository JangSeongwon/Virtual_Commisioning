
import rospy
import os
import threading, time
import sys
import math
import pandas as pd
import numpy as np

from Config import Config
from Gocator_Interface import GocatorInterface
from sklearn.neighbors import NearestNeighbors

sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../common/imp")) ) # get import path : DSR_ROBOT.py 

# for single robot 
ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "m1509"
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *


if __name__ == "__main__":

    Count = 1
    Entire_Recording = []

    while True:
        Recorder = []        

        Measurement = get_current_posj()   
        Recorder = np.array(Measurement)
        print("Count:", Count, " Joints:", Measurement)
        Entire_Recording.append(Recorder)

        user_input = input("PRESS ENTER FOR Continue Running, 다시 측정시 're',종료 시 'done' 입력 : ") 
        # print(Entire_Recording)
        if user_input.lower() == "done":
            break


        if user_input.lower() == "re":
            Entire_Recording.pop()
            continue
        Count += 1

    df = pd.DataFrame(Entire_Recording)
    save_dir = "~/catkin_ws/src/doosan-robot/JSW/"
    save_path = os.path.join(save_dir, "Robot_Joints_Verification.xlsx")
    df.to_excel(save_path, index = False)
    
