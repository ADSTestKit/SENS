
import os, sys
import traceback
import numpy as np

from customized_utils2 import ProcessProvider, Plotter, print_txt
from npc_planner.passenger_car_kinematic_model.PassengerCarKinematicModel import PassengerCarKinematicModel
from npc_planner.speed_optimizer.leastsq import SpeedOptimize


def get_ext_state(pred_route, last_pos, last_ext, display=False):
    pred_x = np.insert(pred_route[:, 0], 0, last_pos[0])
    pred_y = np.insert(pred_route[:, 1], 0, last_pos[1])
    pred_t = np.arange(len(pred_x)) / 10 # * 0.1 will intro error

    t0_vx = last_ext[0]
    t0_vy = last_ext[1]

    funcs_x, _ = SpeedOptimize(pred_x, pred_t, t0_v=t0_vx, noise=False, m=9)
    funcs_y, _ = SpeedOptimize(pred_y, pred_t, t0_v=t0_vy, noise=False, m=9)

    if False:
        import pylab as plt
        fig, axs = plt.subplots(2, 2,)
        t_show = np.linspace(pred_t[0], pred_t[-1], 1000)
        for i, (pred_, funcs) in enumerate([(pred_x, funcs_x), (pred_y, funcs_y)]):
            ax = axs[0, i]
            ax.plot(pred_t, pred_, 'bo', label='pred_xy')
            ax.plot(t_show, funcs[0](t_show), color='g',linestyle='--',marker='',label='leastsq')
            ax.legend()

            ax = axs[1, i]
            ax.plot(t_show, funcs[1](t_show), color='g',linestyle='--',marker='',label='leastsq')
            ax.legend()

    # plt.show()
    pred_vx = funcs_x[1](pred_t)
    pred_vy = funcs_y[1](pred_t)
    pred_yaw = np.arctan2(pred_vy, pred_vx) / np.pi * 180 # cmath.phase() / np.pi * 180
    return np.stack((pred_vx, pred_vy, pred_yaw), axis=1)[1:]


class PredPlanner():
    '''
        controlled planner
    '''
    num_futr = 5
    def __init__(self, model, kwps, num_vhs):
        '''
        # param num_vhs: num of vhs
        '''
        self.model = model
        self.num_past = 20 # depend on model >= 2 general
        # self.num_futr # depend on controler >= 2 general

        self.num_vhs = num_vhs # inlcude ego
        # record all wps for each vhs (including ego); X, Y, Vx, Vy, Theta = state
        self.wps = [np.array([[-100.0] * 3]) for x in range(num_vhs)] # X, Y, t(step, int, initial t < -self.num_past)
        self.wps_ext = [np.array([[-100.0] * 3]) for x in range(num_vhs)] # Vx, Vy, Theta
        self.plan_id = set()

        self.kwps = kwps # inlcude ego, np[[dx, dy, TODO dt]] t[vid[kwps]]
        self.kwps_handler = [PassengerCarKinematicModel() for x in range(num_vhs)]
        self.TOTAL_STEP = len(kwps)

        self.displayer = Plotter(title='PredPlanner', size=720, scale=12.0)
        self.centre = np.array([0.0, 0.0])

    def update_map(self, city=None, _map=None):
        '''
        usage example:
            # obj_map = planner.model._map
            # obj_map.update_grp(city=self._wmap.name, _wmap=self._wmap)
            # planner.update_map(_map=obj_map)
        '''
        self.model.update_map(city, _map)

    def plan(self, t):
        '''
        input hist: all vehicles past 20 points
        output future: all vehicles past 20 points and future x points x<=30, from self.t
        '''
        self.t = t # t start from future 0 using plan for 3 steps, i.e., t t+1 t+2, instead of plan 3times, use past to plan t+2 (20 in LanGCN), because all know plan [t:t+1] for npcs
        # update real info
        hist, hist_time = self._get_hist(t, num_past=self.num_past, num_futr=self.num_futr)
        preds_vid = [(vid, h) for vid, h in enumerate(hist) if h.shape[0]==self.num_past]
        data = dict()
        hist_time = t # TODO: use hist as input, then as outputs, so outputs may > 20
        if preds_vid and hist_time >= 0:
            hist = [h for vid, h in preds_vid] # -15 -> -1 for the first time
            preds_vid_should = [vid for vid, h in preds_vid]

            data['trajs'] = [wps[:,:2] for wps in hist]
            # self.displayer.dots(self.centre, np.concatenate(data['trajs']), (255, 0, 255))
            data['steps'] = [wps[:,-1]-hist_time+20 for wps in hist] # t -> 18 in LanGCN [np.arange(self.num_past)] * len(hist) 
            data['city'] = self.model._map.cities[0] # TODO:

            # TODO: distributed data for parallel; pids -> range(len(pids))
            data['argo_id'] = ProcessProvider.get_id()
            datas = [data]
            datas = self.model.pred(datas) # dict for each argo_id; preds (vids, 30 ,2) # preds_vid (vids,)
            data = datas[ProcessProvider.get_id()]

            NUM_FUTU = 20 # usually > self.num_futr and < 20 (LaneGCN)
            k = 0 
            tmp = np.arange(hist_time, hist_time+NUM_FUTU).reshape(NUM_FUTU, 1)
            data['preds'] = [np.concatenate((data[p_vid][k][:NUM_FUTU], tmp), axis=1) if p_vid in data else 
                             np.concatenate((np.tile(hist[p_vid][-1,:2], (NUM_FUTU, 1)), tmp), axis=1)
                             for p_vid, vid in enumerate(preds_vid_should)]  # (vids, NUM_FUTU ,3)
            data['preds_vid'] = preds_vid_should
            data['hist_time'] = hist_time
            assert len(data['preds']) == len(data['preds_vid'])

        self._update_wps(data)

    def _get_hist(self, hist_time, num_past=20, num_futr=3):
        '''output hist: vehicles in record past 20 points
        '''
        cut_t = hist_time + num_futr
        futr = [wps[(wps[:,-1] >= hist_time) & (wps[:,-1] < cut_t)] for wps in self.wps if wps.shape[0] >= num_past]
        futr = [fr.shape[0] for fr in futr]
        futr_offset = min(futr) if futr else 0

        if futr_offset < num_futr:
            hist_time += futr_offset
        else: # after set plan and keep a length
            return [], hist_time

        cut_t = hist_time - num_past
        hist = [wps[(wps[:,-1] >= cut_t) & (wps[:,-1] < hist_time)] for wps in self.wps]

        return hist, hist_time

    def _update_wps(self, data=None):
        kinematic_list = []
        if self.t in self.kwps: # step: [(u1, u2, dt), ] TODO from (step_pre, u1, u2, dstep)
            for vid, kwp_ in self.kwps[self.t].items(): # TODO: dt is time
                if vid not in self.plan_id:
                    continue

                kinematic_list.append(vid)
                if kwp_ is None or len(kwp_) == 0:
                    print_txt(f'vehicle {vid} has setting from kinematic ahead')
                    continue
                for tmp_t in range(self.t+1, kwp_[-1]+1):
                    self.kwps[tmp_t][vid] = None
                kwp = kwp_.copy()
                kwp[-1] += self.num_futr
                kwp[-1] /= 10

                wps = self.wps[vid]
                wps_ext = self.wps_ext[vid]
                index_ = wps[:,-1] < self.t
                (X, Y, T) = wps[index_][-1]
                V = np.linalg.norm(wps_ext[index_][-1][:2])
                Theta = wps_ext[index_][-1][2]

                _han = self.kwps_handler[vid]
                futr_states = _han.run(cur_state=np.array([X, Y, V, Theta * np.pi / 180, T / 10]), 
                                       u1_u2_ts=[kwp])

                self.wps[vid] = np.concatenate((wps[index_], 
                                                np.concatenate((futr_states[:,:2], futr_states[:,-1:] * 10), axis=1)), 
                                                axis=0)
                self.wps_ext[vid] = np.concatenate((wps_ext[index_], 
                                                    np.stack((futr_states[:,2] * np.cos(futr_states[:,3]), 
                                                              futr_states[:,2] * np.sin(futr_states[:,3]), 
                                                              futr_states[:,3] / np.pi * 180), axis=1)), 
                                                    axis=0)
                self.displayer.dots(self.centre, futr_states[:,:2], (255, 125, 0))

            # del self.kwps[self.t]
            print_txt("self.t in self.kwps")

        if 'preds_vid' in data:
            for i, vid in enumerate(data['preds_vid']): # i = np.where(data['preds_vid']==vid)[0][0]
                if vid in kinematic_list:
                    print_txt(f'vehicle {vid} has setting from kinematic, not use predplan')
                    continue

                wps = self.wps[vid]
                index_ = wps[:,-1] < data['hist_time']
                self.wps[vid] = np.concatenate((wps[index_], data['preds'][i]), axis=0)

                wps_ext = self.wps_ext[vid]
                self.wps_ext[vid] = np.concatenate((wps_ext[index_], get_ext_state(data['preds'][i], wps[index_][-1], wps_ext[index_][-1], display=vid==1)), axis=0)
                self.displayer.dots(self.centre, data['preds'][i][:,:2], (255, 255, 0))

            print_txt("'preds_vid' in data")

        self.displayer.show()

    def send_reqs(self, ):
        '''send wps for npcs'''
        wps_npcs = self.wps
        wps_npcs_ext = self.wps_ext
        return wps_npcs, wps_npcs_ext

    def _pull_motion_history(self, motion_hist):
        '''
            pull and merge with wps
            real close to plan, diff due to control and others
        '''
        for vid, hist in enumerate(motion_hist):
            self.pull_hist(vid, hist)

    def stop_vehicle(self, vid, t):
        index_num = sum(self.wps[vid][:,-1] < t)
        try:
            self.wps[vid][index_num:] = self.wps[vid][index_num-1]
            self.wps_ext[vid][index_num:] = self.wps_ext[vid][index_num-1]
            self.wps[vid][index_num:,-1] = np.arange(len(self.wps[vid][index_num:,-1])) + t
            self.wps_ext[vid][index_num:,:-1] *= 0.0
        except IndexError as e:
            NUM_FUTU = 20
            self.wps[vid] = np.concatenate((self.wps[vid], np.tile(self.wps[vid][-1], (NUM_FUTU,1))), axis=0)
            self.wps_ext[vid] = np.concatenate((self.wps_ext[vid], np.tile(self.wps_ext[vid][-1], (NUM_FUTU,1))), axis=0)
            self.wps[vid][index_num:,-1] = np.arange(len(self.wps[vid][index_num:,-1])) + t
            self.wps_ext[vid][index_num:,:-1] *= 0.0
        except Exception as e:
            traceback.print_exc()
            print(e)

    def pull_hist(self, vid, hist: np.ndarray):
        hist_ext = hist[-3:].copy() # Vx, Vy, Theta
        hist = hist[:3].copy() # X, Y, t
        self.displayer.dot(self.centre, hist[:2], (0, 0, 255))
        # if self.t == 0:
        #     print('dbu')

        try: # for each vhs (TODO: record ego[0])
            index_num = sum(self.wps[vid][:,-1] < hist[-1])
            orig = self.wps[vid][index_num].copy()
            orig2 = self.wps_ext[vid][index_num].copy()
            self.wps[vid][index_num] = hist
            self.wps_ext[vid][index_num] = hist_ext

            assert orig[2]-hist[2] == 0.0, "assert timeoff == 0"
            posoff = np.linalg.norm(orig[:2]-hist[:2])
            if posoff > 0.3 and vid == 1 and self.t >= 0 and np.linalg.norm(hist_ext[:2]) > 0.1: # TODO: V > 0.05
                print("hist[2], vid, posoff", )
                print(hist[2], vid, posoff, sep='\t')
                self.displayer.dot(self.centre, orig[:2], (255, 0, 0))
                self.displayer.show()
                print('warning posoff!') 
            yawoff = (orig2[2] - hist_ext[2]) / 180 * np.pi
            if abs(np.arctan2(np.sin(yawoff), np.cos(yawoff))) > np.pi / 6 and vid == 1 and self.t >= 0:
                print('warning yawoff!')
            speoff = np.linalg.norm(orig2[:2]-hist_ext[:2])
            if speoff > 0.3 and vid == 1 and self.t >= 0:
                print("hist[2], vid, speoff", )
                print(hist[2], vid, speoff, sep='\t')
                # self.displayer.dot(self.centre, orig[:2], (255, 0, 0))
                # self.displayer.show()
                print('warning speedoff!')

        except IndexError as e:
            assert self.t < 0, f'warning: update vehicle {vid} without setting, still init?'
            if self.wps[vid].size >= 3:
                self.wps[vid] = np.concatenate((self.wps[vid], hist.reshape(1,-1)), axis=0)
                self.wps_ext[vid] = np.concatenate((self.wps_ext[vid], hist_ext.reshape(1,-1)), axis=0)
            else:
                self.wps[vid] = hist.reshape(1,-1)
                self.wps_ext[vid] = hist_ext.reshape(1,-1)
            self.plan_id.add(vid)
        except Exception as e:
            traceback.print_exc()
            print(e)

    def set_centre(self, centre):
        self.centre = centre

    def set_num_past(self, num_past):
        self.num_past = max(min(self.num_past, num_past), 3)
        self.t = -self.num_past

    def stop_check(self):
        '''
        '''
        if self.t >= self.TOTAL_STEP:
            return True
        return False


class VehicleModel():
    def __init__(self, ):
        self.model = None

    def run(self, inputs):
        outputs = self._run_batch(inputs)
        outputs_pro = self._post_process(self, outputs)
        return outputs_pro

    def _initialize(self, ):
        # load and train
        self.model = None
        # test model using a sample
        inputs = self._get_sample_inputs()
        outputs = self._run(inputs)
        pass

    def _run(self, inputs):
        assert inputs
        outputs = None
        return outputs

    def _run_batch(self, inputs):
        outputs = None
        return outputs

    def _get_sample_inputs(self, ):
        inputs = None
        return inputs

    def _post_process(self, outputs):
        outputs_pro = None
        return outputs_pro


