
import atexit
import glob
import itertools
import operator
import os, sys
import random
import pickle
import pathlib
import subprocess
from multiprocessing import Process
import time
import math
import traceback
import pywt
import numpy as np
import psutil
import socket
from datetime import datetime
from collections import Counter
from scipy import sparse
from scipy.interpolate import make_interp_spline
import shapely
from shapely.geometry import Polygon, MultiPoint, Point
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.path as mplPath


def calc_semantic(ego_trj=None, npcs_trj=None, info1=None, ax=None, TEST_I=[]):
    ego_trj, npcs_trj = cut_extra(ego_trj, npcs_trj)

    color = (random.random(),random.random(),random.random())

    stop_index = int(max(sum( np.abs(ego_trj['v_traj']-ego_trj['v_traj'][-1])>0.001 )))
    close_index1 = np.argmin(np.linalg.norm(ego_trj['v_traj']-npcs_trj['vehicle.tesla.model3']['v_traj'], axis=1))
    close_index2 = np.argmin(np.linalg.norm(ego_trj['v_traj']-npcs_trj['vehicle.tesla.cybertruck']['v_traj'], axis=1))
    domin_index = max((stop_index, close_index1, close_index2))

    ego_pos_fin = ego_trj['v_traj'][domin_index]
    ego_yaw_fin = ego_trj['v_yaw'][domin_index] # draw_pot(ego_pos_fin, ax, yaw=ego_yaw_fin)
    model3_pos_fin = npcs_trj['vehicle.tesla.model3']['v_traj'][domin_index]
    model3_yaw_fin = npcs_trj['vehicle.tesla.model3']['v_yaw'][domin_index]
    cybertruck_pos_fin = npcs_trj['vehicle.tesla.cybertruck']['v_traj'][domin_index]
    cybertruck_yaw_fin = npcs_trj['vehicle.tesla.cybertruck']['v_yaw'][domin_index]

    if info1['route_completion'] < 99.0: # crash_or_uncompleted
        ego_bbox = get_bbox(create_transform(ego_pos_fin[0], ego_pos_fin[1], yaw=ego_yaw_fin), vehicles_extent['vehicle.lincoln.mkz2017'])
        model3_bbox = get_bbox(create_transform(model3_pos_fin[0], model3_pos_fin[1], yaw=model3_yaw_fin), vehicles_extent['vehicle.tesla.model3'])
        cybertruck_bbox = get_bbox(create_transform(cybertruck_pos_fin[0], cybertruck_pos_fin[1], yaw=cybertruck_yaw_fin), vehicles_extent['vehicle.tesla.cybertruck'])

        a = np.array([[p.x, p.y] for p in ego_bbox])
        b = np.array([[p.x, p.y] for p in model3_bbox])
        c = np.array([[p.x, p.y] for p in cybertruck_bbox])
        # if i in TEST_I:
        #     plt_trj({'ego_trj':ego_trj, 'npcs_trj':npcs_trj}, ax, color)
        #     ax.plot(a.T[0], a.T[1], c=color)
        #     ax.plot(b.T[0], b.T[1], c=color)
        #     ax.plot(c.T[0], c.T[1], c=color)

        poly1 = Polygon(a).convex_hull
        poly2 = Polygon(b).convex_hull
        poly3 = Polygon(c).convex_hull

        if poly1.distance(poly2) < poly1.distance(poly3):
            npc_closet = 2 # 'model3'
        else:
            npc_closet = 1 # 'cybertruck'
    else:
        npc_closet = 0 # 'none'

        poly1 = None
        poly2 = None
        poly3 = None

    angle_rela = np.arctan2(ego_pos_fin[1]-model3_pos_fin[1], ego_pos_fin[0]-model3_pos_fin[0])*180/np.pi \
                - model3_yaw_fin
    angle_rela2 = np.arctan2(ego_pos_fin[1]-cybertruck_pos_fin[1], ego_pos_fin[0]-cybertruck_pos_fin[0])*180/np.pi \
                - cybertruck_yaw_fin

    maneuver1 = get_maneuver_sequence(npcs_trj['vehicle.tesla.model3'], first_len=domin_index)
    maneuver2 = get_maneuver_sequence(npcs_trj['vehicle.tesla.cybertruck'], first_len=domin_index)

    if ax is not None:
        draw_pot(ego_pos_fin, ax, yaw=ego_yaw_fin, bbox=poly1)
        draw_pot(model3_pos_fin, ax, yaw=model3_yaw_fin, bbox=poly2, maneuver=maneuver1)
        draw_pot(cybertruck_pos_fin, ax, yaw=cybertruck_yaw_fin, bbox=poly3, maneuver=maneuver2)

    semantic = dict()
    semantic['npc_closet'] = npc_closet
    semantic['angle_rela_model3'] = ((30+angle_rela)%360)//60
    semantic['angle_rela_cybertruck'] = ((30+angle_rela2)%360)//60
    semantic['maneuver_model3'] = maneuver1
    semantic['maneuver_cybertruck'] = maneuver2
    semantic['vector'] = (npc_closet, ((30+angle_rela)%360)//60, ((30+angle_rela2)%360)//60)+maneuver1+maneuver2

    return semantic, ego_trj, npcs_trj
