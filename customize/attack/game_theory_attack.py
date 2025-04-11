import math
import tensorflow

class Game:
    def __init__(self, vehicle, vehicle_list, score=0, state='0'):
        """
        Initialize the Game instance

        Parameters
        ----------
        vehicle: The attacker
        vehicle_list: The platoon
        score: Initial score, default is 0
        state: Current state of the game, default is '0'
        """
        self.vehicle = vehicle
        self.score = score
        self.state = state
        self.vehicle_list = vehicle_list
        self.world = vehicle.get_world()
        self.v_v = 0  # velocity of the attacker
        self.p_v = 0  # average velocity of the platoon
        self.v_n = 0  # predicted next average velocity of the platoon
        self.d_n = 0  # predicted next average distance between the vehicles in the platoon
        self.control = None  # current control state of the attacker
        self.collision = False  # collision status
        self.model = tensorflow.keras.models.load_model('vehicle_model')  # load ML models for predictions
        self.cross_vehicle = None  # vehicle used for lane-changing calculations
        self.substate = None  # substate within certain states
        self.r_a = 0  # value for decision-making
        x = self.get_platoon_middle()  # precompute platoon middle location

    def update_score(self):
        """Increment the score by 1"""
        self.score += 1

    def update_state(self):
        """
        Update the state of the game based on vehicle velocity,
        platoon state, relative positions, and scores
        """
        # calculate the velocity of vehicle
        v_x = self.vehicle.get_velocity().x
        v_y = self.vehicle.get_velocity().y
        v_z = self.vehicle.get_velocity().z
        self.v_v = math.sqrt(v_x ** 2 + v_y ** 2 + v_z ** 2)

        # calculate the average velocity of platoon
        p_x = 0
        p_y = 0
        p_z = 0
        p_num = 0
        x = []
        leader = None
        follower = None

        for vehicle in self.vehicle_list:
            p_x += vehicle.get_velocity().x
            p_y += vehicle.get_velocity().y
            p_z += vehicle.get_velocity().z
            p_num += 1
            x.append(vehicle.get_location().x)

        self.p_v = math.sqrt((p_x / p_num) ** 2 + (p_y / p_num) ** 2 + (p_z / p_num) ** 2)
        for vehicle in self.vehicle_list:
            if max(x) == vehicle.get_location().x:
                leader = vehicle
            elif min(x) == vehicle.get_location().x:
                follower = vehicle

        if self.get_platoon_middle() is not None:
            self.state = '1'
            if self.score > 10 and self.state == '1':
                self.state = '2'

            vehicle_waypoint = self.world.get_map().get_waypoint(self.vehicle.get_location())
            platoon_waypoint = self.world.get_map().get_waypoint(self.vehicle_list[0].get_location())

            if self.state == '2' and vehicle_waypoint.lane_id == platoon_waypoint.lane_id and self.score > 25:
                self.state = '3'
                if self.vehicle.get_location().x >= leader.get_location().x:
                    self.substate = 'leader'  # leader
                elif self.vehicle.get_location().x <= follower.get_location().x:
                    self.substate = 'follower'  # follower
                else:
                    self.substate = 'middle'  # middle
            if self.state == '3' and vehicle_waypoint.lane_id == platoon_waypoint.lane_id and self.score > 40:
                self.state = '4'


    def get_platoon_middle(self):
        """
        Determine the middle vehicle in the platoon and validate
        if it's on the same road as the attacker

        Returns:
            The location of the middle vehicle if valid, otherwise None
        """
        # find the middle vehicle of the platoon
        platoon_list = [vehicle.get_location().x for vehicle in self.vehicle_list]
        state = ''
        fir_x = 0

        platoon_middle = platoon_list[len(self.vehicle_list) // 2 - 1]

        platoon = next((v for v in self.vehicle_list if v.get_location().x == platoon_middle), None)
        if platoon is None:
            return None

        self.control = platoon.get_control()

        # check road compatibility
        waypoint_platoon = self.world.get_map().get_waypoint(platoon.get_location())
        waypoint_vehicle = self.world.get_map().get_waypoint(self.vehicle.get_location())
        if waypoint_platoon.road_id != waypoint_vehicle.road_id:
            return None

        v_middle = self.cal_distance(self.vehicle, platoon)
        v_leader = self.cal_distance(self.vehicle, self.vehicle_list[0])
        v_follower = self.cal_distance(self.vehicle, self.vehicle_list[len(self.vehicle_list) - 1])

        # todo: please change attack method here
        """ Leader Attack
        state = 'leader'
        self.cross_vehicle = self.vehicle_list[0]
        location = self.cross_vehicle.get_location()
        location.x += self.d_n / 1.5
        return location
        """

        """ Mid-Platoon Attack
        state = 'middle'
        self.cross_vehicle = platoon
        location = platoon.get_location()
        location.x -= self.d_n / 3
        return location
        """

        """ Follower Attack
        state = 'follower'
        self.cross_vehicle = self.vehicle_list[len(self.vehicle_list) - 1]
        location = self.cross_vehicle.get_location()
        location.x -= self.d_n / 1.5
        return location
        """

    def get_distance(self):
        """
        Calculate the distance between the attacker and the platoon middle

        Returns:
            A tuple of the distance whether the vehicle is forward relative to the platoon
        """
        if self.get_platoon_middle() is None:
            dx = 100
            return dx

        platoon_location = self.get_platoon_middle()
        vehicle_location = self.vehicle.get_location()
        forward = True

        dx = vehicle_location.x - platoon_location.x
        dy = vehicle_location.y - platoon_location.y
        dz = platoon_location.z - vehicle_location.z
        d = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

        comp_x = self.vehicle_list[0].get_location().x - self.vehicle_list[1].get_location().x

        forward = comp_x * dx > 0

        return d, forward

    def cal_distance(self, vehicle1, vehicle2):
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
    
    def cal_speed(self, vehicle):
        v_x = vehicle.get_velocity().x
        v_y = vehicle.get_velocity().y
        v_z = vehicle.get_velocity().z
        speed = math.sqrt(v_x**2 + v_y**2 + v_z**2)
        return speed
    
    def cal_acc(self, vehicle):
        v_x = vehicle.get_acceleration().x
        v_y = vehicle.get_acceleration().y
        v_z = vehicle.get_acceleration().z
        speed = math.sqrt(v_x**2 + v_y**2 + v_z**2)
        return speed
    
    def cal_ra(self, p_s, d_t, v_p):
        self.r_a = 0.6 * p_s + 0.9 * d_t + 0.9 * v_p
        return self.r_a

    def punish_cost(self):
        """
        Calculate the penalty cost based on state and velocity mismatch

        Returns:
            The calculated cost value
        """
        if self.state != '1':
            return 0

        d, forward = self.get_distance()
        return 0.05 * abs((self.v_v - self.p_v)) + 0.1 * abs(d)

    def predict_behavior(self):
        """
        Predict the next velocity and distance using the loaded model

        Returns:
            A tuple of predicted velocity and distance
        """
        distances = [vehicle.get_location() for vehicle in self.vehicle_list]
        avg_distance = sum(self.cal_distance(self.vehicle_list[i], self.vehicle_list[i+1]) for i in range(len(distances) - 1)) / (len(self.vehicle_list) - 1)

        # model predict next v and next d from previous t, b, v, and d
        prediction = self.model.predict([[self.control.throttle, self.control.brake, self.p_v, avg_distance]])
        return prediction[0, 0], prediction[0, 1]

    def strategy_making(self, control, waypoint):
        """
        Core decision-making logic for vehicle movement based on game state

        Args:
            control: The control object to modify throttle and brake values
            waypoint: The target waypoint for navigation

        Returns:
            Updated control and waypoint
        """

        """
        State 0: move the vehicle approaching the middle
        State 1: Adjust vehicle velocity and distance relative to the platoon
        State 2: Change lane to join the platoon
        State 3: adjustments within the platoon
        State 4: Dynamic abnormal behaviours
        """
        self.v_n, self.d_n = self.predict_behavior()
        if self.state == '1' and self.get_platoon_middle() is not None:
            self.handle_state_1(control)
        elif self.state == '2':
            self.handle_state_2(waypoint)
        elif self.state == '3':
            self.handle_state_3(control, waypoint)
        elif self.state == '4':
            self.handle_state_4(control)

        return control, waypoint

    def handle_state_1(self, control):
        """Handle state 1 logic"""
        threshold = 0.5  # the threshold to decide whether to change the state
        if self.punish_cost() >= threshold:
            d, forward = self.get_distance()
            if self.v_v > self.p_v + 1:
                control.brake = min(1.0, 0.1 * (self.v_v - self.p_v))
            if d > 5 and forward:
                control.brake = min(control.brake+0.02*(d-5), 1)
            elif d < 5 and forward:
                control.brake = 0
            elif d > 5 and not forward:
                control.brake = 0
                control.throttle = min(self.control.throttle+0.02*(d-5), 1)
            else:
                control.brake = 0
                control.throttle = min(self.control.throttle + 0.02, 1)
            self.score = 0
        else:
            control.throttle = self.control.throttle
            control.brake = self.control.brake
            self.update_score()

    def handle_state_2(self, waypoint):
        """
        Handle state 2 logic
        Selection of waypoint is self-defined
        """
        cross_vehicle = self.cross_vehicle

        waypoint.y = self.vehicle_list[0].get_location().y
        waypoint.x += 10

        waypoint_platoon = self.world.get_map().get_waypoint(cross_vehicle.get_location())
        waypoint_vehicle = self.world.get_map().get_waypoint(self.vehicle.get_location())

        if waypoint_platoon.lane_id == waypoint_vehicle.lane_id:
            self.update_score()

    def handle_state_3(self, control, waypoint):
        waypoint.y = self.vehicle_list[0].get_location().y

        if self.substate == 'leader':
            self.handle_leader_state(control)
        elif self.substate == 'follower':
            self.handle_follower_state(control)
        else:
            self.handle_middle_state(control)

    def handle_state_4(self, control):
        if self.substate == 'leader':
            self.handle_final_leader_state(control)
        elif self.substate == 'follower':
            self.handle_final_follower_state(control)
        else:
            self.handle_final_middle_state(control)

    def handle_leader_state(self, control):
        """Handle leader logic"""
        fir_v = self.vehicle_list[0]
        delta_d = self.cal_distance(self.vehicle, fir_v)

        if control.brake == 0:
            control.brake = 0.5

        if delta_d < self.d_n:
            control.throttle = min(control.throttle + 0.2, 1)
            control.brake = 0
        else:
            control.brake = min(control.brake + 0.02, 1)
            control.throttle = max(control.throttle - 0.02, 0)
        self.update_score()

    def handle_follower_state(self, control):
        """Handle follower logic"""
        fir_v = self.vehicle_list[len(self.vehicle_list) - 1]
        delta_d = self.cal_distance(fir_v, self.vehicle)

        if control.throttle == 0:
            control.throttle = self.control.throttle
        if control.brake == 1:
            control.brake = self.control.brake

        if delta_d < self.d_n:
            control.brake = min(control.brake + 0.02, 1)
            control.throttle = max(control.throttle - 0.02, 0)
        else:
            if control.throttle + 0.02 <= 1:
                control.throttle += 0.02
                control.brake = 0.0
            if delta_d > 1.5 * self.d_n:
                control.throttle = 1.0
                control.brake = 0.0

        if self.d_n - 1 <= delta_d <= self.d_n + 1:
            self.update_score()

    def handle_middle_state(self, control):
        """Handle middle logic"""
        fir_v = self.cross_vehicle
        delta_d = self.cal_distance(fir_v, self.vehicle)
        las_v = next(
            (self.vehicle_list[i + 1] for i in range(len(self.vehicle_list) - 1) if fir_v == self.vehicle_list[i]),
            None)
        d_delta = self.cal_distance(self.vehicle, las_v)

        if control.throttle == 0:
            control.throttle = self.control.throttle
        if control.brake == 1:
            control.brake = self.control.brake

        if delta_d < self.d_n:
            control.brake = min(control.brake + 0.02, 1)
            control.throttle = max(control.throttle - 0.02, 0)
        else:
            control.throttle = min(control.throttle + 0.02, 1)
            control.brake = 0.0
        if d_delta <= 6.5:
            if control.throttle + 0.03 <= 1:
                control.throttle += 0.03
                control.brake = 0.0

        if self.d_n - 1 <= delta_d <= self.d_n + 1 or abs(delta_d - d_delta) <= 1:
            self.update_score()

    def handle_final_leader_state(self, control):
        fir_v = self.vehicle_list[0]
        delta_d = self.cal_distance(self.vehicle, fir_v)

        if self.cal_ra(delta_d, 1e-6, 1e-6) > (self.d_n):
            if control.throttle == 1:
                control.throttle = 0.5
            if control.brake == 0:
                control.brake = 0.2
        else:
            if delta_d >= 0.5 * self.d_n:
                if control.brake + 0.2 <= 1:
                    control.brake += 0.2
                elif control.brake + 0.1 <= 1:
                    control.brake += 0.1
            else:
                if control.throttle + 0.2 <= 1:
                    control.throttle += 0.2
                elif control.throttle + 0.1 <= 1:
                    control.throttle += 0.1

    def handle_final_follower_state(self, control):
        fir_v = self.vehicle_list[len(self.vehicle_list) - 1]
        delta_d = self.cal_distance(fir_v, self.vehicle)

        if self.cal_ra(1e-6, 1e-6, delta_d) > (self.d_n - 1) / self.d_n:
            if control.throttle == 0:
                control.throttle = self.control.throttle
            if control.brake == 1:
                control.brake = self.control.brake
        else:
            if delta_d > 6.5:
                if control.throttle + 0.03 >= 1:
                    control.throttle += 0.03
                control.brake = 0
            else:
                control.brake = min(control.brake + 0.03, 1)
                control.throttle = max(control.throttle - 0.03, 0)

    def handle_final_middle_state(self, control):
        fir_v = self.cross_vehicle
        las_v = next(
            (self.vehicle_list[i + 1] for i in range(len(self.vehicle_list) - 1) if fir_v == self.vehicle_list[i]),
            None)
        d_delta = self.cal_distance(self.vehicle, las_v)
        delta_d = self.cal_distance(fir_v, self.vehicle)
        
        if control.throttle == 0:
            control.throttle = self.control.throttle
        if control.brake == 1:
            control.brake = self.control.brake
        
        if self.cal_ra(1e-6, delta_d/self.v_v, 1e-6) < self.d_n/self.v_n or delta_d < d_delta:
            control.brake = min(control.brake + 0.03, 1)
            control.throttle = max(control.throttle - 0.03, 0)
        else:
            control.throttle = min(control.throttle + 0.03, 1)
            control.brake = 0
        if delta_d > 6.5:
            control.throttle = min(control.throttle + 0.03, 1)
            control.brake = 0

class GameManager:
    _game_instances = {}

    @classmethod
    def get_game(cls, vehicle, vehicle_list):
        if vehicle not in cls._game_instances:
            cls._game_instances[vehicle] = Game(vehicle, vehicle_list)
        return cls._game_instances[vehicle]
