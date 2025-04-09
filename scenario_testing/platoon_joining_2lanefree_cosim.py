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

    finally:

        eval_manager.evaluate()
        scenario_manager.close()

        for platoon in platoon_list:
            platoon.destroy()

        for v in single_cav_list:
            v.destroy()
