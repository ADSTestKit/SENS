
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


from npc_planner.passenger_car_kinematic_model.PassengerCarKinematicModel import PassengerCarKinematicModel
from npc_planner.speed_optimizer.utils import csv2list, plot_embedding_2d, plot_embedding_3d


class OfflineAnalyzer():
    def __init__(self, args, args_dict, sim_specific_arguments, history_bucket_root=None):
        self.history_bucket = history_bucket_root if history_bucket_root \
                         else args.parent_folder
        simulation, self.scenario_model = prepare_simulation(args, args_dict, sim_specific_arguments, static_analysis=True)
        simulation = None

    def run(self, candis, x, total_i):
        if len(candis) != 2:
            print_debug(f'{total_i}: 2 candis in same class expected!')
            return False, None

        run_info_candis = [self.run_info_list[_id] for _id in candis]
        thecla = run_info_candis[0]['class']

        # Trajectory Generation
        print_debug(f"{total_i}: OfflineAnalyzer use {run_info_candis[0]['total_i']} {run_info_candis[1]['total_i']} in class {thecla}/{len(self.class_index_global_dict)}")
        ego_trjs = [{
            k: np.pad(v[:240], ((0, 240-len(v[:240])), (0,0)), 'edge') if len(v.shape) == 2 else
                np.pad(v[:240], (0, 240-len(v[:240])), 'edge')
            for k,v in run_info_candi['ego_trj'].items()
        } for run_info_candi in run_info_candis]
        ego_trj = ego_trjs[0]
        # s_ = time.time()
        run_info_curr = self.scenario_model.Offline_Trajectory_Generation(x, total_i, ego_trj, self.npc_info_lib) # like run_simulation
        # print(f'time Offline_Trajectory_Generation: {time.time()-s_}')
        if run_info_curr['has_run_static'] == 0:
            return False, None

        # Qualified?
        cla, is_friend = classify_scenario1(
            run_info_curr, self.run_info_list, self.class_index_global_dict, 
            has_classes_target=thecla
        )
        print_debug(f"{total_i}: different scenario semantic {[cla, thecla]} {is_friend} ({[run_info_curr['scenario_semantic']['vector'], run_info_candis[0]['scenario_semantic']['vector']]})")
        if cla == None:
            return False, None

        run_info_curr['class'] = cla
        with open(self.history_bucket / f"cur_info_{total_i}_{total_i}_{int(time.time())}.pickle", 'wb') as f_out:
            pickle.dump([run_info_curr], f_out)

        print_debug(f"{total_i}: OfflineAnalyzer success in class {cla}!")
        return True, run_info_curr

    def check_hypo(self, run_info_curr, cla):
        same_class_index = self.class_index_global_dict[cla]
        run_info_same_class = [self.run_info_list[_id] for _id in same_class_index]

        flag = is_in_local_region(run_info_curr, run_info_same_class)

        # fig = plt.figure(f'xy{cla}')
        # ax = fig.add_subplot(1, 1, 1)
        # ax.set_aspect(1)
        # ax.xaxis.set_ticks_position('top')
        # ax.invert_yaxis()
        # ax.set(xlabel='x', ylabel='z in UE, y in Carla')
        # ax.xaxis.set_label_position('top')

        # for info1 in run_info_same_class:
        #     color = (random.random(),random.random(),random.random())
        #     plt_trj(info1, ax, color)
        # plt_trj(run_info_curr, ax, 'r')

        # fig.savefig(f"tmp/cla{cla}.png")
        # plt.close(f'xy{cla}')
        return flag

    def found_candis(self, x: np.ndarray):
        (
            self.run_info_list, 
            all_test_cases,
            self.class_index_global_dict, # {cls: [ind1, ind2, ind3, ...], ...}
            self.npc_info_lib,
            cur_files,
            none_class_list,
            failed_list
        ) = get_histoty(self.history_bucket)
        if len(self.class_index_global_dict) == 0: # not enough history data
            print_debug('not enough history data')
            return []
        run_info_list = self.run_info_list

        num_clost = 1
        self.try_hypovec = get_npcs_ms(x.astype(float)).reshape(-1,)
        all_test_cases_clas = np.array([run_info['class'] for run_info in run_info_list]) # (n,)
        all_hypovec = np.array([run_info['hypovec'] for run_info in run_info_list])
        d_all = all_hypovec - self.try_hypovec
        d_all = np.linalg.norm(d_all, axis=1)

        closet_index = np.argsort(d_all)
        closet_clas = all_test_cases_clas[closet_index]
        closet_dist = d_all[closet_index]

        tmpc = []
        for clo_cla in list(closet_clas):
            tmpc.append(clo_cla)
            if len(tmpc) - len(set(tmpc)) == num_clost:
                break
        if len(tmpc) - len(set(tmpc)) < num_clost:
            clo_cla = -10

        min_2_index = np.where(np.array(tmpc)==clo_cla)[0]

        return list(closet_index[min_2_index])



