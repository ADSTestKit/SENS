
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


class HistoryCollector(Process):
    def __init__(self, args, history_collector_dict, history_bucket_root=None):
        self.history_bucket = history_bucket_root if history_bucket_root \
                         else args.parent_folder
        self.history_bucket = pathlib.Path(self.history_bucket)
        self.history_collector_dict = history_collector_dict

        self.cur_files = set()
        self.run_info_list = [] # save
        self.class_index_global_dict = dict() # save
        self.none_class_list = []
        self.npc_info_lib = dict()

        self.failed_list = []
        self.pop_size = args.pop_size
        super().__init__()

    def run(self):
        print("HistoryCollector(Process) run")
        (
            self.run_info_list, 
            all_test_cases,
            self.class_index_global_dict, # {cls: [ind1, ind2, ind3, ...], ...}
            self.npc_info_lib,
            self.cur_files,
            self.none_class_list,
            self.failed_list
        ) = get_histoty(self.history_bucket)
        while(self.history_collector_dict['collect']):
            timestamp = int(time.time())
            cur_files = set()
            for f in self.history_bucket.glob('cur_info_*.pickle'):
                name_list = f.stem.split('_')
                if len(name_list) != 5:
                    continue
                if timestamp - int(name_list[-1]) < 5:
                    continue
                cur_files.add(f) # all current possible files

            # to_abspath
            cur_files_tmp = list(cur_files)
            def to_abspath(cur_file):
                return cur_file.absolute()
            cur_files_tmp = map(to_abspath, cur_files_tmp)
            cur_files_tmp = list(cur_files_tmp)
            cur_files = set(cur_files_tmp)

            cur_files_add = cur_files.difference(self.cur_files)
            if len(cur_files_add) == 0:
                time.sleep(1.0)
                continue
            self.cur_files = cur_files

            cur_files_add = list(cur_files_add)
            cur_files_add.sort()
            run_info_list_add = []
            for f in cur_files_add:
                with open(f, 'rb') as f_info:
                    run_info_list_add.extend(pickle.load(f_info))

            if len(run_info_list_add) == 0:
                time.sleep(1.0)
                continue

            # update info
            self.scenario_classify(run_info_list_add)

            # info lib using npc info in run_info_list
            for run_info_ in run_info_list_add:
                if 'vehicle_list' in run_info_:
                    vehicle_list = run_info_['vehicle_list']
                    npcs_trj = run_info_['npcs_trj']
                    self.npc_info_lib = {
                        model: {vehicle_list[model]: _trj}
                        for model, _trj in npcs_trj.items()
                    }


            total_num = len(self.none_class_list) + len(self.run_info_list) + len(self.failed_list)
            timestamp = int(time.time())
            with open(self.history_bucket / f'run_info_list_0_{total_num-1}_{timestamp}.pickle', 'wb') as f_out:
                pickle.dump(self.run_info_list, f_out)

        end()

    def scenario_classify(self, run_info_list_add):
        '''
        add ['class'] for each run_inf, 
        then add to run_info_list and get index
        class_index_global_dict {cla: [ind1, ind2, ind3, ...], ...}
        none_class_list [inf_w/o_cla, ...]
        '''
        run_info_list = self.run_info_list
        class_index_global_dict = self.class_index_global_dict
        none_class_list = self.none_class_list

        for run_inf in run_info_list_add:
            if 'scenario_semantic' not in run_inf or len(run_inf['scenario_semantic'])==0:
                self.failed_list.append(run_inf)
                continue
            cla, is_friend = classify_scenario1(run_inf, run_info_list, class_index_global_dict) if 'class' not in run_inf else (run_inf['class'], True)
            if cla:
                run_inf['class'] = cla
                class_index_global_dict[cla].append(len(run_info_list))
                run_info_list.append(run_inf)
            else: # None, 0, ''
                none_class_list.append(run_inf)

        run_infs_with_class, none_class_list = classify_scenario2(none_class_list, list(class_index_global_dict.keys()))
        for run_inf in run_infs_with_class:
            cla = run_inf['class']
            if cla not in class_index_global_dict:
                class_index_global_dict[cla] = []
            class_index_global_dict[cla].append(len(run_info_list))
            run_info_list.append(run_inf)

        self.run_info_list = run_info_list
        self.class_index_global_dict = class_index_global_dict
        self.none_class_list = none_class_list



