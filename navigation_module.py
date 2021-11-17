import pickle
import numpy as np
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET
import copy
import os 
import bezier
import os, sys
import numba
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import sumolib
from sumolib import checkBinary
import traci
dirname = os.path.dirname(__file__)
SUMOCFG_DIR = dirname + "/map/Map4_Tsinghua_Intersection/configuration.sumocfg"
SUMO_BINARY = checkBinary('sumo-gui')
import warnings
warnings.filterwarnings("ignore")

dis_interval = 0.5
num_ref_points = 10
step_time = 0.1
DeadDistance_base = 20

class Navigation():
    def __init__(self, agentID:str):
        start="gneE664"
        end="chengfu_out"
        self.in_junction = False
        self.net_path = R'map\Map4_Tsinghua_Intersection\grid-map.net.xml'
        self.rou_path = R'map\Map4_Tsinghua_Intersection\grid-map-traffic.rou.xml'
        self.trip_path = R'navigation\trips.xml'
        self.result_path = R'navigation\result.rou.xml'

        self._init_traci_without_gui()
        self.edge_list = self._get_edges(start, end)
        self.update_rou_xml()

        self._prepare_map_info()
        self._get_static_navi_info()
        traci.close()
        self._init_traci()
        self.current_edge = self.edge_list[0]
        self.current_index = self.best_number_dict[self.current_edge][0]
        
    def _init_traci_without_gui(self):
        seed = 10
        SUMO_BINARY = checkBinary('sumo')
        port = sumolib.miscutils.getFreeSocketPort()
        traci.start(
            [SUMO_BINARY, "-c", SUMOCFG_DIR,
                "--step-length", str(step_time),
                "--lateral-resolution", "3.75",
                "--no-warnings",
                "--no-step-log",
                '--seed', str(int(seed))
                ], port=port, numRetries=5) 

    def _init_traci(self):
        seed = 10
        port = sumolib.miscutils.getFreeSocketPort()
        traci.start(
            [SUMO_BINARY, "-c", SUMOCFG_DIR,
                "--step-length", str(step_time),
                "--lateral-resolution", "3.75",
                "--no-warnings",
                "--no-step-log",
                '--seed', str(int(seed))
                ], port=port, numRetries=5)
    
    def _get_edges(self, start, end):
        tree = ET.parse(self.trip_path)
        root = tree.getroot()
        route_info = root.find('trip')
        route_info.attrib['from'] = str(start)
        route_info.attrib['to'] = str(end)
        tree=ET.ElementTree(root)
        tree.write(self.trip_path)
        route_command = R'duarouter --route-files ' + self.trip_path + ' --net-file ' + self.net_path + ' --output-file ' + self.result_path
        os.system(route_command)        # using duarouter to get routing result
        
        tree = ET.parse(self.result_path)
        root = tree.getroot()
        if root.find('vehicle') == None:
            print('NO route available!')
            edge_list = []
        else:
            edge_list = root.find('vehicle').find('route').attrib['edges'].split()
        return edge_list          

    def update_rou_xml(self):
        tree = ET.parse(self.rou_path)
        root = tree.getroot()
        rou_str = ''
        for edge in self.edge_list:
            rou_str += edge
            rou_str += ' '
        root.find('route').attrib['edges'] = rou_str
        tree=ET.ElementTree(root)
        tree.write(self.rou_path)

    def _get_next_lane_list(self, current_edge):
        current_edge_info = self._R2RwDL_short[current_edge]
        for edge in current_edge_info:
            if edge in self.edge_list:
                next_edge = edge
                if next_edge == self.edge_list[-1]:
                    lane_list = self.edge_lane_numbers[next_edge]
                else:
                    next_edge_info = self._R2RwDL_short[next_edge]
                    for next_next_edge in next_edge_info:
                        if next_next_edge in self.edge_list:
                            direction, lane_list = next_edge_info[next_next_edge][0], next_edge_info[next_next_edge][1:]
        lane_list = self._expand_lane(lane_list, next_edge)
        lane_list = self._expand_lane(lane_list, next_edge)
        self.next_lane_list = lane_list
        self.next_edge = next_edge
        return lane_list, next_edge

    def _get_bezier_outdict(self, lane_list, next_edge):
        bezier_output_dict = {}
        bezier_feature_output_dict = {}
        current_lane_points = self._LInfo[self.current_lane_id]['shape']
        phi_current_lane = np.arctan2(current_lane_points[1][1] - current_lane_points[0][1], current_lane_points[1][0] - current_lane_points[0][0])
        for lane in lane_list:
            lane_id = f'{next_edge}_{lane}'
            lane_points = self._LInfo[lane_id]['shape']
            phi_next_lane = np.arctan2(lane_points[1][1] - lane_points[0][1], lane_points[1][0] - lane_points[0][0])
            bezier_output_dict[lane_id], bezier_feature_output_dict[lane_id] = get_bezier_connect(current_lane_points[1][0], current_lane_points[1][1], phi_current_lane, lane_points[0][0], lane_points[0][1], phi_next_lane)
        self.output_dict = bezier_output_dict
        self.feature_output_dict = bezier_feature_output_dict

    def _check_clr_lane(self, lane_index, edgeID, x, y): # current, left, right lane
        if lane_index in self.edge_lane_numbers[edgeID]:
            lane_id = f'{edgeID}_{lane_index}'
            points = self._LInfo[lane_id]['shape']
            lane_refs = self._get_ref_points(points, x, y)
            lane_points_array_list = []
            for lane_ref in lane_refs:
                lane_x, lane_y, lane_phi = lane_ref
                lane_points_array = get_straight_points_array(np.array(lane_x), np.array(lane_y), np.array(lane_phi))
                lane_points_array_list.append(lane_points_array)
        else:
            lane_points_array_list = None
        return lane_points_array_list

    def update(self, x, y, edgeID, laneIndex, ahead_lane_length):
        DeadDistance = ahead_lane_length - DeadDistance_base
        if edgeID in self.edge_list:
            self.current_edge = edgeID
            self.current_index = laneIndex
            
        self.current_lane_id = f'{self.current_edge}_{self.current_index}'

        if edgeID in self.edge_list and DeadDistance > 0:
            self.in_junction = False
            current_points_array = self._check_clr_lane(laneIndex, edgeID, x, y)
            left_points_array = self._check_clr_lane(laneIndex-1, edgeID, x, y)
            right_points_array = self._check_clr_lane(laneIndex+1, edgeID, x, y)

            best_lane_dis = 10      # just a big enough numebr
            for num in self.best_number_dict[edgeID]:
                if abs(num - laneIndex) < abs(best_lane_dis):
                    best_lane_dis = num - laneIndex

            final_output_dict = {
                'in_junciton':False,
                'points':[current_points_array, left_points_array, right_points_array],
                'best_lane':best_lane_dis,  #
                'DeadDistance':DeadDistance
            }
        else:  # return multi bezier curves
            lane_list, next_edge = self._get_next_lane_list(self.current_edge)
            self._get_bezier_outdict(lane_list, next_edge)
            if self.in_junction == False and (edgeID in self.edge_list):
                current_points_array = self._check_clr_lane(laneIndex, edgeID, x, y)
                points_array_list = []
                feature_points_array_list = []
                for k,v in self.feature_output_dict.items():    
                    feature_points_array_list.append(self.feature_output_dict[k])
                final_output_dict = {
                    'points':[current_points_array, None, None],
                    'best_lane':None,
                    'dead_dist':None,
                    'bezier_points':[feature_points_array_list, None, None]
                }
            else:
                self.in_junction = True
                bezier_output_dict = self._deal_begin(x, y, num_ref_points = num_ref_points)
                
                points_array_list = []
                feature_points_array_list = []
                for k,v in bezier_output_dict.items():
                    points_array_list.append(bezier_output_dict[k])
                for k,v in self.feature_output_dict.items():    
                    feature_points_array_list.append(self.feature_output_dict[k])

                final_output_dict = {
                    'points':[points_array_list, None, None],
                    'best_lane':None,
                    'dead_dist':None,
                    'bezier_points':[feature_points_array_list, None, None]
                }
        return final_output_dict['points']

    def _deal_begin(self, cur_x, cur_y, num_ref_points = num_ref_points):
        output_dict = {}
        for k, v in self.output_dict.items():
            x, y, phi = v
            dis_array = np.sqrt((np.array(x) - cur_x)**2 + (np.array(y) - cur_y)**2)
            index_start = np.argmin(dis_array)

            points_array = np.ones((4, num_ref_points))
            if index_start + num_ref_points > len(phi):
                n2 = index_start + num_ref_points - len(phi)
                n1 = num_ref_points - n2
                next_lane_points = self._LInfo[k]['shape']
                n_ref = self._get_ref_points(next_lane_points, next_lane_points[0][0], next_lane_points[0][1], num_ref_points=n2)
                n_x, n_y, n_phi = n_ref[0][0], n_ref[0][1], n_ref[0][2]
                points_array[0,:n1] = np.array(x[index_start:])
                points_array[0,n1:] = np.array(n_x)
                points_array[1,:n1] = np.array(y[index_start:])
                points_array[1,n1:] = np.array(n_y)
                points_array[2,:n1] = np.array(phi[index_start:])
                points_array[2,n1:] = np.array(n_phi)
            else:
                points_array[0,:] = np.array(x[index_start:index_start + num_ref_points])
                points_array[1,:] = np.array(y[index_start:index_start + num_ref_points])
                points_array[2,:] = np.array(phi[index_start:index_start + num_ref_points])
            points_array[3,:] *= 5

            output_dict[k] = points_array
        return output_dict
 
    def _expand_lane(self, lane_list, edgeID):
        total_lane_list = self.edge_lane_numbers[edgeID]
        if lane_list != total_lane_list:     
            new_list = []
            if lane_list != total_lane_list:     
                for lane in total_lane_list:
                    for cur_lane in lane_list:
                        new_list.append(cur_lane)
                        if abs(lane-cur_lane) <= 1:
                            new_list.append(lane)
                new_list = list(set(new_list))
                new_list.sort()
            return new_list
        else:
            return lane_list
        
    def _get_ref_points(self, points_list, cur_x, cur_y, num_ref_points = num_ref_points):
        # 输出的坐标点列表
        x_l = []  
        y_l = []
        for i in range(len(points_list)-1):
            x1, y1 = points_list[i]
            x2, y2 = points_list[i+1]
            tmp_x_list, tmp_y_list = get_points_list_from_two_points(x1, y1, x2, y2)
            x_l += tmp_x_list
            y_l += tmp_y_list
        x = np.array(x_l)
        y = np.array(y_l)
        phi = np.arctan2(y[1:]-y[:-1],x[1:]-x[:-1])
        phi = np.concatenate((phi, phi[-1:]), axis=0)
        dis_array = np.sqrt((x - cur_x)**2 + (y - cur_y)**2)
        index_start = np.argmin(dis_array)
        if num_ref_points == 'auto':
            return x, y, phi
        elif index_start+num_ref_points <= len(phi):
            return [(x[index_start:index_start+num_ref_points], y[index_start:index_start+num_ref_points], \
                    phi[index_start:index_start+num_ref_points])]
        else:
            lane_list, next_edge = self._get_next_lane_list(self.current_edge)
            self._get_bezier_outdict(lane_list, next_edge)  # can get self.output_dict
            n2 = index_start + num_ref_points - len(phi)
            n1 = num_ref_points - n2
            refs_list = []
            for k, v in self.output_dict.items():
                n_x, n_y, n_phi = v
                tem_x = list(x[index_start:]) + list(n_x[:n2])
                tem_y = list(y[index_start:]) + list(n_y[:n2])
                tem_phi = list(phi[index_start:]) + list(n_phi[:n2])
                refs_list.append((tem_x, tem_y, tem_phi))
            return refs_list

    def _get_static_navi_info(self):
        self._RL2RaL_short = {}
        for edg in self.edge_list:
            self._RL2RaL_short[edg] = self._RL2RaL[edg]
        self._RL2RaL_short_copy = copy.deepcopy(self._RL2RaL_short)
        for i,edg in enumerate(self.edge_list):
            for k,v in self._RL2RaL[edg].items():
                if not ((i == len(self.edge_list)-1) or (list(v.keys())[0]==self.edge_list[i+1])):
                    del self._RL2RaL_short_copy[edg][k]

        self._R2RwDL_short = {}
        for edg in self.edge_list:
            self._R2RwDL_short[edg] = self._R2RwDL[edg]

    def _prepare_map_info(self):
        global_var_list = ['_LInfo',\
                        '_R2RwDL',\
                        '_RL2RaL']
        path_list = []
        for var in global_var_list:
            path_list.append('navigation\mapinfo'+var+'.pickle')
        var_dict = {}
        for i, path in enumerate(path_list):
            with open(path, 'rb') as f:
                tem = pickle.load(f)
                var_dict[global_var_list[i]] = tem
        self._LInfo=var_dict['_LInfo']
        self._R2RwDL=var_dict['_R2RwDL']['passenger']
        self._RL2RaL=var_dict['_RL2RaL']['passenger']

        self.best_number_dict = {}
        self.edge_lane_numbers= {}
        num_edge = len(self.edge_list)
        for i in range(num_edge):
            best_number_list = []
            for lane_num in self._RL2RaL[self.edge_list[i]]:
                if i < num_edge - 1:    
                    if list(self._RL2RaL[self.edge_list[i]][lane_num].keys())[0]==self.edge_list[i+1]:
                        best_number_list.append(lane_num)
                else:
                    best_number_list.append(lane_num)
            self.best_number_dict[self.edge_list[i]] = best_number_list[::-1]
            self.edge_lane_numbers[self.edge_list[i]] = list(self._RL2RaL[self.edge_list[i]].keys())[::-1]
        # print(self.edge_lane_numbers)
        # print(self.best_number_dict)

def get_straight_points_array(x_list, y_list, phi_list):
    points_array = np.ones((4, num_ref_points )) # v=5m/s
    points_array[0,:] = x_list 
    points_array[1,:] = y_list
    points_array[2,:] = phi_list
    points_array[3,:] *= 5.
    return points_array

def get_bezier_connect(x1, y1, phi1, x4, y4, phi4):
    delta_x = abs(x4 - x1)
    delta_y = abs(y4 - y1)
    x2 = x1 + delta_x / 3 * np.cos(phi1)
    y2 = y1 + delta_y / 3 * np.sin(phi1)
    x3 = x4 - delta_x / 3 * np.cos(phi4)
    y3 = y4 - delta_y / 3 * np.sin(phi4)

    node = np.array([[ x1,  x2, x3, x4], 
                    [y1, y2, y3, y4]])
    curve = bezier.Curve(node, degree=3)
    s_vals = np.linspace(0, 1.0, 10000)
    curve_lane = curve.evaluate_multi(s_vals).astype(np.float32)
    curve_len = np.sum(np.sqrt(np.square(curve_lane[0][1:] - curve_lane[0][:-1]) +
                            np.square(curve_lane[1][1:] - curve_lane[1][:-1])))
    s_vals = np.linspace(0, 1.0, int(curve_len / (dis_interval)), endpoint=False)
    curve_lane = curve.evaluate_multi(s_vals).astype(np.float32)

    x_l = np.array(curve_lane[0])
    y_l = np.array(curve_lane[1])
    phi_l = np.arctan2(y_l[1:] - y_l[:-1],
                        x_l[1:] - x_l[:-1])
    phi_expand = np.concatenate((phi_l, np.array([phi_l[-1]])))

    feature_points = np.array([[x1, x2, x3, x4],[y1, y2, y3, y4]])
    return (x_l, y_l, phi_expand), feature_points

@numba.njit()
def get_points_list_from_two_points(x1, y1, x2, y2):
    def get_length(x1, y1, x2, y2):
        return np.sqrt((x1-x2)**2 + (y1-y2)**2)
    def get_route_points_num(length, interval=dis_interval):
        return np.round(length/interval)
    length = get_length(x1,y1, x2, y2)
    num = get_route_points_num(length, interval=dis_interval)
    cut_list = np.arange(0, num, 1)
    x_list = (x2 - x1) * cut_list/num + x1
    y_list = (y2 - y1) * cut_list/num + y1
    return list(x_list), list(y_list)

def scale_phi(phi):
    return np.mod(phi+np.pi,2*np.pi)-np.pi

def get_update_info(x, y):
    edgeID, lanePosition, laneIndex = traci.simulation.convertRoad(x, y)
    length = traci.lane.getLength(f'{edgeID}_{laneIndex}')
    ahead_lane_length = length - lanePosition
    return edgeID, laneIndex, ahead_lane_length
if __name__ == "__main__":
    vehID=0
    navi = Navigation(vehID)
    num = 1200
    import time
    time_list = []
    
    traci.vehicle.add(vehID=str(vehID), routeID='self_route', departLane = 2, typeID="car_4")
    traci.simulationStep()
    x, y = traci.vehicle.getPosition(str(vehID))
    for i in range(int(num)):        
        edgeID, laneIndex, ahead_lane_length = get_update_info(x, y)
        start = time.perf_counter_ns()
        out_dict = navi.update(x, y, edgeID, laneIndex, ahead_lane_length)
        end = time.perf_counter_ns()
        time_list.append(float(end - start)/10**6)
        points = out_dict
        x = points[0][0][0][4]
        y = points[0][0][1][4]
        phi = points[0][0][2][4]
        angle_in_sumo = scale_phi(-phi + np.pi/2)*180/np.pi
        if edgeID==navi.edge_list[-1] and ahead_lane_length < num_ref_points*dis_interval + DeadDistance_base:
            print('finished')
            break
        traci.vehicle.moveToXY(vehID=str(vehID), edgeID=edgeID,lane = laneIndex, x=x, y=y, angle=angle_in_sumo)
        traci.simulationStep()
    traci.close()


    # np.savetxt('time_list', time_list, delimiter=',')
    # print('############')
    # print(np.mean(time_list))
    # plt.plot(time_list)
    # plt.ylabel('update time[ms]')
    # plt.xlabel('steps')
    # plt.show()





