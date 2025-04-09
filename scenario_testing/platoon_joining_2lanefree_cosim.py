# -*- coding: utf-8 -*-
"""
Scenario testing: merging vehicle joining a platoon in the
customized 2-lane freeway simplified map sorely with co-sim
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import os
import time

import carla

import opencda.scenario_testing.utils.cosim_api as sim_api
import opencda.scenario_testing.utils.customized_map_api as map_api
from opencda.core.common.cav_world import CavWorld
from opencda.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
from opencda.scenario_testing.utils.yaml_utils import add_current_time
import numpy as np
import math
import random
'''middle
throttle_path_a = os.path.join(os.getcwd(), 'middle', 'throttle_data_a.npy')
brake_path_a = os.path.join(os.getcwd(), 'middle', 'brake_data_a.npy')
distance_path_af = os.path.join(os.getcwd(), 'middle', 'distance_data_af.npy')
distance_path_ab = os.path.join(os.getcwd(), 'middle', 'distance_data_ab.npy')
speed_path_a = os.path.join(os.getcwd(), 'middle', 'speed_data_a.npy')
location_path_a = os.path.join(os.getcwd(), 'middle', 'location_data_a.npy')

throttle_path_pf = os.path.join(os.getcwd(), 'middle', 'throttle_data_pf.npy')
throttle_path_pb = os.path.join(os.getcwd(), 'middle', 'throttle_data_pb.npy')
brake_path_pf = os.path.join(os.getcwd(), 'middle', 'brake_data_pf.npy')
brake_path_pb = os.path.join(os.getcwd(), 'middle', 'brake_data_pb.npy')
distance_path_p = os.path.join(os.getcwd(), 'middle', 'distance_data_p.npy')
speed_path_pf = os.path.join(os.getcwd(), 'middle', 'speed_data_pf.npy')
speed_path_pb = os.path.join(os.getcwd(), 'middle', 'speed_data_pb.npy')
location_path_pf = os.path.join(os.getcwd(), 'middle', 'location_data_pf.npy')
location_path_pb = os.path.join(os.getcwd(), 'middle', 'location_data_pb.npy')
'''
'''leader
throttle_path_a = os.path.join(os.getcwd(), 'leader', 'throttle_data_a.npy')
brake_path_a = os.path.join(os.getcwd(), 'leader', 'brake_data_a.npy')
distance_path_a = os.path.join(os.getcwd(), 'leader', 'distance_data_a.npy')
speed_path_a = os.path.join(os.getcwd(), 'leader', 'speed_data_a.npy')
location_path_a = os.path.join(os.getcwd(), 'leader', 'location_data_a.npy')

throttle_path_p = os.path.join(os.getcwd(), 'leader', 'throttle_data_p.npy')
brake_path_p = os.path.join(os.getcwd(), 'leader', 'brake_data_p.npy')
distance_path_p = os.path.join(os.getcwd(), 'leader', 'distance_data_p.npy')
speed_path_p = os.path.join(os.getcwd(), 'leader', 'speed_data_p.npy')
location_path_p = os.path.join(os.getcwd(), 'leader', 'location_data_p.npy')
'''
'''follower
throttle_path_a = os.path.join(os.getcwd(), 'follower', 'throttle_data_a.npy')
brake_path_a = os.path.join(os.getcwd(), 'follower', 'brake_data_a.npy')
distance_path_a = os.path.join(os.getcwd(), 'follower', 'distance_data_a.npy')
speed_path_a = os.path.join(os.getcwd(), 'follower', 'speed_data_a.npy')
location_path_a = os.path.join(os.getcwd(), 'follower', 'location_data_a.npy')

throttle_path_p = os.path.join(os.getcwd(), 'follower', 'throttle_data_p.npy')
brake_path_p = os.path.join(os.getcwd(), 'follower', 'brake_data_p.npy')
distance_path_p = os.path.join(os.getcwd(), 'follower', 'distance_data_p.npy')
speed_path_p = os.path.join(os.getcwd(), 'follower', 'speed_data_p.npy')
location_path_p = os.path.join(os.getcwd(), 'follower', 'location_data_p.npy')
'''
dist12_ = os.path.join(os.getcwd(), 'leader', 'dist12.npy')
speed12_ = os.path.join(os.getcwd(), 'leader', 'speed12.npy')
acc12_ = os.path.join(os.getcwd(), 'leader', 'acc12.npy')
speed_ = os.path.join(os.getcwd(), 'leader', 'speed.npy')

def cal_distance(vehicle1, vehicle2):
    """
    Calculate the Euclidean distance between two vehicles

    Args:
        vehicle1: First vehicle instance
        vehicle2: Second vehicle instance

    Returns:
        The distance between the two vehicles
    """
    dx = vehicle1.get_location().x - vehicle2.get_location().x
    dy = vehicle1.get_location().y - vehicle2.get_location().y
    dz = vehicle1.get_location().z - vehicle2.get_location().z

    return math.sqrt(dx**2 + dy**2 + dz**2)
    
def cal_speed(vehicle):
    v_x = vehicle.get_velocity().x
    v_y = vehicle.get_velocity().y
    v_z = vehicle.get_velocity().z
    speed = math.sqrt(v_x**2 + v_y**2 + v_z**2)
    return speed
    
def cal_acc(vehicle):
    v_x = vehicle.get_acceleration().x
    v_y = vehicle.get_acceleration().y
    v_z = vehicle.get_acceleration().z
    speed = math.sqrt(v_x**2 + v_y**2 + v_z**2)
    return speed

def run_scenario(opt, scenario_params):
    try:
        scenario_params = add_current_time(scenario_params)

        # create CAV world
        cav_world = CavWorld(opt.apply_ml)

        # sumo conifg file path
        current_path = os.path.dirname(os.path.realpath(__file__))

        xodr_path = os.path.join(current_path,
                                 '../assets/2lane_freeway_simplified/'
                                 '2lane_freeway_simplified.xodr')
        sumo_cfg = os.path.join(current_path,
                                '../assets/2lane_freeway_simplified')

        # create co-simulation scenario manager
        scenario_manager = \
            sim_api.CoScenarioManager(scenario_params,
                                      opt.apply_ml,
                                      opt.version,
                                      xodr_path=xodr_path,
                                      cav_world=cav_world,
                                      sumo_file_parent_path=sumo_cfg)

        # create platoon members
        platoon_list = \
            scenario_manager.create_platoon_manager(
                map_helper=map_api.spawn_helper_2lanefree,
                data_dump=False)

        single_cav_list = \
            scenario_manager.create_vehicle_manager(application=['platooning'],
                                                    map_helper=map_api.
                                                    spawn_helper_2lanefree)

        # create evaluation manager
        eval_manager = \
            EvaluationManager(scenario_manager.cav_world,
                              script_name='single_2lanefree_cosim',
                              current_time=scenario_params['current_time'])

        spectator = scenario_manager.world.get_spectator()
        spectator_vehicle = platoon_list[0].vehicle_manager_list[1].vehicle
        platoon_add = False
        vehicle_list = platoon_list[0].vehicle_manager_list

        throttle_data_a = np.array([])
        throttle_data_p = np.array([])
        throttle_data_pf = np.array([])
        throttle_data_pb = np.array([])
        brake_data_a = np.array([])
        brake_data_p = np.array([])
        brake_data_pf = np.array([])
        brake_data_pb = np.array([])
        speed_data_a = np.array([])
        speed_data_p = np.array([])
        speed_data_pf = np.array([])
        speed_data_pb = np.array([])
        dist_data_af = np.array([])
        dist_data_ab = np.array([])
        dist_data_a = np.array([])
        dist_data_p = np.array([])
        loc_data_a = np.array([])
        loc_data_p = np.array([])
        loc_data_pf = np.array([])
        loc_data_pb = np.array([])
        dist12 = np.array([])
        speed12 = np.array([])
        acc12 = np.array([])
        speed = np.array([])

        while True:
            # simulation tick
            # scenario_manager.tick()
            # todo: changed simulation tick
            scenario_manager.tick(platoon_list, single_cav_list, platoon_add)

            transform = spectator_vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location +
                                                    carla.Location(z=80),
                                                    carla.Rotation(pitch=-90)))

            throttle_p = 0
            brake_p = 0
            speed_p = 0
            dis = []
            dis_p = 0
            count_p = 0

            for platoon in platoon_list:
                platoon.update_information()
                platoon.run_step()
            
            '''middle
            vehicle1 = vehicle_list[1].vehicle
            vehicle2 = vehicle_list[2].vehicle

            vx1 = vehicle1.get_velocity().x
            vy1 = vehicle1.get_velocity().y
            vz1 = vehicle1.get_velocity().z
            v1 = math.sqrt(vx1 ** 2 + vy1 ** 2 + vz1 ** 2)
            x1 = vehicle1.get_location().x
            y1 = vehicle1.get_location().y
            z1 = vehicle1.get_location().z
            loc1 = [x1, y1, z1]

            vx2 = vehicle2.get_velocity().x
            vy2 = vehicle2.get_velocity().y
            vz2 = vehicle2.get_velocity().z
            v2 = math.sqrt(vx2 ** 2 + vy2 ** 2 + vz2 ** 2)
            x2 = vehicle2.get_location().x
            y2 = vehicle2.get_location().y
            z2 = vehicle2.get_location().z
            loc2 = [x2, y2, z2]

            throttle_data_pf = np.append(throttle_data_pf, vehicle1.get_control().throttle)
            throttle_data_pb = np.append(throttle_data_pb, vehicle2.get_control().throttle)
            brake_data_pf = np.append(brake_data_pf, vehicle1.get_control().brake)
            brake_data_pb = np.append(brake_data_pb, vehicle2.get_control().brake)
            speed_data_pf = np.append(speed_data_pf, v1)
            speed_data_pb = np.append(speed_data_pb, v2)
            loc_data_pf = np.append(loc_data_pf, loc1)
            loc_data_pb = np.append(loc_data_pb, loc2)

            dx = vehicle_list[0].vehicle.get_location().x - vehicle_list[1].vehicle.get_location().x
            dy = vehicle_list[0].vehicle.get_location().y - vehicle_list[1].vehicle.get_location().y
            dz = vehicle_list[0].vehicle.get_location().z - vehicle_list[1].vehicle.get_location().z
            d = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
            dist_data_p = np.append(dist_data_p, d)

            np.save(throttle_path_pf, throttle_data_pf)
            np.save(throttle_path_pb, throttle_data_pb)
            np.save(brake_path_pf, brake_data_pf)
            np.save(brake_path_pb, brake_data_pb)
            np.save(speed_path_pf, speed_data_pf)
            np.save(speed_path_pb, speed_data_pb)
            np.save(location_path_pf, loc_data_pf)
            np.save(location_path_pb, loc_data_pb)
            np.save(distance_path_p, dist_data_p)
            '''
            '''leader and follower
            for vehicle in vehicle_list:
                throttle_p += vehicle.vehicle.get_control().throttle
                brake_p += vehicle.vehicle.get_control().brake

                v_x = vehicle.vehicle.get_velocity().x
                v_y = vehicle.vehicle.get_velocity().y
                v_z = vehicle.vehicle.get_velocity().z
                speed = math.sqrt(v_x**2 + v_y**2 + v_z**2)
                speed_p += speed

                if count_p == 0:
                    x = vehicle.vehicle.get_location().x
                    y = vehicle.vehicle.get_location().y
                    z = vehicle.vehicle.get_location().z
                    loc_data_p = np.append(loc_data_p, (x, y, z))

                dis.append(vehicle.vehicle.get_location())

                count_p += 1

            throttle_data_p = np.append(throttle_data_p, throttle_p / count_p)
            brake_data_p = np.append(brake_data_p, brake_p / count_p)
            speed_data_p = np.append(speed_data_p, speed_p / count_p)

            for i in range(len(dis)-1):
                dx = dis[i].x - dis[i+1].x
                dy = dis[i].y - dis[i+1].y
                dz = dis[i].z - dis[i+1].z
                dis_p += math.sqrt(dx**2 + dy**2 + dz**2)
            dist_data_p = np.append(dist_data_p, dis_p / count_p)

            np.save(throttle_path_p, throttle_data_p)
            np.save(brake_path_p, brake_data_p)
            np.save(speed_path_p, speed_data_p)
            np.save(distance_path_p, dist_data_p)
            np.save(location_path_p, loc_data_p)
            #'''
            
            for i, single_cav in enumerate(single_cav_list):
                single_cav.update_info()
                """ when the attack is in the middle or behind the platoon
                waypoint_platoon = scenario_manager.world.get_map().get_waypoint(vehicle_list[0].vehicle.get_location())
                waypoint_vehicle = scenario_manager.world.get_map().get_waypoint(single_cav.vehicle.get_location())
                if waypoint_platoon.lane_id == waypoint_vehicle.lane_id and platoon_add is False:
                    platoon_list[0].add_member(single_cav)
                    platoon_list[0].set_member(single_cav, 4) # the number is the order of the attack in the platoon
                    platoon_list[0].update_member_order()
                    platoon_add = True
                """
                control = single_cav.run_step()
                single_cav.vehicle.apply_control(control)
                """leader and follower
                throttle_data_a = np.append(throttle_data_a, single_cav.vehicle.get_control().throttle)
                brake_data_a = np.append(brake_data_a, single_cav.vehicle.get_control().brake)

                v_x = single_cav.vehicle.get_velocity().x
                v_y = single_cav.vehicle.get_velocity().y
                v_z = single_cav.vehicle.get_velocity().z
                speed = math.sqrt(v_x ** 2 + v_y ** 2 + v_z ** 2)
                speed_data_a = np.append(speed_data_a, speed)

                x = single_cav.vehicle.get_location().x
                y = single_cav.vehicle.get_location().y
                z = single_cav.vehicle.get_location().z
                loc_data_a = np.append(loc_data_a, (x, y, z))

                dx = single_cav.vehicle.get_location().x - dis[1].x
                dy = single_cav.vehicle.get_location().y - dis[1].y
                dz = single_cav.vehicle.get_location().z - dis[1].z
                d = math.sqrt(dx * dx + dy * dy + dz * dz)
                dist_data_a = np.append(dist_data_a, d)

                np.save(distance_path_a, dist_data_a)
                np.save(throttle_path_a, throttle_data_a)
                np.save(brake_path_a, brake_data_a)
                np.save(speed_path_a, speed_data_a)
                np.save(location_path_a, loc_data_a)
                # middle
                dx1 = single_cav.vehicle.get_location().x - vehicle_list[1].vehicle.get_location().x
                dy1 = single_cav.vehicle.get_location().y - vehicle_list[1].vehicle.get_location().y
                dz1 = single_cav.vehicle.get_location().z - vehicle_list[1].vehicle.get_location().z
                d1 = math.sqrt(dx1 ** 2 + dy1 ** 2 + dz1 ** 2)

                dx2 = single_cav.vehicle.get_location().x - vehicle_list[2].vehicle.get_location().x
                dy2 = single_cav.vehicle.get_location().y - vehicle_list[2].vehicle.get_location().y
                dz2 = single_cav.vehicle.get_location().z - vehicle_list[2].vehicle.get_location().z
                d2 = math.sqrt(dx2 ** 2 + dy2 ** 2 + dz2 ** 2)

                dist_data_af = np.append(dist_data_af, d1)
                dist_data_ab = np.append(dist_data_ab, d2)
                np.save(distance_path_af, dist_data_af)
                np.save(distance_path_ab, dist_data_ab)
                """

    finally:

        eval_manager.evaluate()
        scenario_manager.close()

        for platoon in platoon_list:
            platoon.destroy()

        for v in single_cav_list:
            v.destroy()
