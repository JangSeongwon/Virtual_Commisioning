
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

Calibration_sphere_pos = [500, 26, 75]
Gocator_height = 74.5

cfg = Config()
go = GocatorInterface(cfg)

if __name__ == "__main__":
    Count = 0
    Entire_Recording = []

    while True:
        Recorder = []        
        Count += 1

        "Gocator Running"
        go.start() 
        print("waiting for the pcd collection... ")
        time.sleep(10.0)
        pcd_list = go.stop() 
        length = len(pcd_list)
        # print(length)

        from Circle_Estimator import CircleEstimator
        radius_recording = []
        for i in range(length):
            circle = CircleEstimator(pcd_list[i]*1000)
            result_df = circle.fit_and_report()
            radius_recording.append(result_df[2])
        Radius = np.mean(radius_recording)
        # print(Radius)

        Z = np.mean(result_df[1])
        real_distance = (130 + Z + Radius+0.02)
        # print(real_distance)

        # user_input = input("Measured Distance")
        # Distance = [user_input]

        Measurement = get_current_posj()   
        # print("Robot Jointvalues ", Count, " : ", Measurement)

        Recorder = np.concatenate(([real_distance], [Radius], Measurement))
        print("Recording", Count, "- Distance: ", real_distance, "Radius: ", Radius, "Joints: ", Measurement)
        Entire_Recording.append(Recorder)

        user_input = input("PRESS ENTER FOR Continue Running, 종료 시 'done' 입력")    
        if user_input.lower() == "done":
            break
        # Solution_space = get_current_solution_space()

    df = pd.DataFrame(Entire_Recording)
    save_dir = "~/catkin_ws/src/doosan-robot/JSW/"
    save_path = os.path.join(save_dir, "Calibration_Sphere_Measurement.xlsx")
    df.to_excel(save_path, index = False)
    
