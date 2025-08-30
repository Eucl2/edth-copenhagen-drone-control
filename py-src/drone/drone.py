import queue
from enum import Enum

import generated.drone_pb2 as pb2
import generated.drone_pb2_grpc as pb2_gprc
import grpc
import time
import os

def PIDCalculation(min_val, max_val, error, orient, i_z=0.0, prev_e_z=0.0, prev_t = None, extraThrottle=0):
    if (orient == 'throttle'):
        hover = ((min_val + max_val) / 2 + 5)
        Kp_z = 10.0
        Ki_z = 1#0.5
        Kd_z = 0.1
    else:
        hover = ((min_val + max_val) / 2)
        Kp_z = 1.2
        Ki_z = 0.06
        Kd_z = 0.2

    i_z_limit = 10.0

    now = time.monotonic()
    dt = (now - prev_t) if prev_t is not None else 0.02
    prev_t = now
    if dt <= 0:
        dt = 1e-3

    P = Kp_z * error
    i_z += error * dt
    if i_z > i_z_limit: i_z = i_z_limit
    if i_z < -i_z_limit: i_z = -i_z_limit
    I = Ki_z * i_z

    # --- D term ---
    D = Kd_z * ((error - prev_e_z) / dt)
    prev_e_z = error

    # PID output is a *delta* around hover
    delta_thr = P + I + D

    u = hover + delta_thr
    pidValue = max(min_val, min(max_val, u))
    #print(f" History: ", i_z, prev_e_z, prev_t)
    return {
        "pidValue": int(round(pidValue)),
        "i_z": i_z,
        "prev_e_z": prev_e_z,
        "prev_t": prev_t
    }

class SimulationState(Enum):
    INIT = 0
    STARTED = 1
    ENDED = 3


class DroneClient:
    simulation_state = SimulationState.INIT

    def __init__(self, host, port, token):
        # instantiate a channel
        self.channel = grpc.insecure_channel("{}:{}".format(host, port))
        # Create a queue for sending messages
        self.send_queue = queue.SimpleQueue()
        # bind the client and the server
        self.stub = pb2_gprc.DroneControllerStub(self.channel)
        # Bind send queue to sending service
        metadata = [("authorization", f"Bearer {token}")]
        self.event_stream = self.stub.DroneConnection(iter(self.send_queue.get, None), metadata=metadata)

        self.time_in_good_zone = 0.0
        self.last_update_time = None
        self.sim_start_time = None

        self.has_taken_off = False

        self.i_z = 0.0
        self.prev_e_z = 0.0
        self.prev_t = None

        self.i_y = 0.0
        self.prev_e_y = 0.0
        self.prev_ty = None

        self.i_x = 0.0
        self.prev_e_x = 0.0
        self.prev_tx = None

        self.phase_tolerance = 4

    def start(self):
        # Start the control loop etc
        self.control_loop()

    def control_drone(self):
        # Get drone position
        print("Start of control_drone")
        print(f"Drone position: ", self.position.x, self.position.y, self.position.z)
        # Target position is defined as a region (maybe you want to compute the center?)
        # Get target maximal position
        max_target_pos = self.goal_region.maximal_point
        print(f"Target maximal position: ", max_target_pos.x, max_target_pos.y, max_target_pos.z)
        # Get target maximal position
        min_target_pos = self.goal_region.minimal_point
        print(f"Target minimal position: ", min_target_pos.x, min_target_pos.y, min_target_pos.z)

        # TODO: Implement your control process here!

        #bigger square, to slow down
        big_min_target_pos = pb2.DroneClientMsg(
            throttle=0, pitch=0, roll=0
        )  # placeholder, we only need coordinates

        big_min_target_pos_x = min_target_pos.x - self.phase_tolerance
        big_min_target_pos_y = min_target_pos.y - self.phase_tolerance
        big_max_target_pos_x = max_target_pos.x + self.phase_tolerance
        big_max_target_pos_y = max_target_pos.y + self.phase_tolerance

        #find center of target region
        x_best = (min_target_pos.x + max_target_pos.x) / 2
        y_best = (min_target_pos.y + max_target_pos.y) / 2
        z_best = (min_target_pos.z + max_target_pos.z) / 2

        #find the error to the center
        error_pos_z = z_best - self.position.z
        error_pos_x = x_best - self.position.x
        error_pos_y = y_best - self.position.y
        print(f"Error position: ", error_pos_x, error_pos_y, error_pos_z)

        # find the error to the laterals
        if (error_pos_x > self.phase_tolerance or error_pos_x < -self.phase_tolerance) and (error_pos_y > self.phase_tolerance or error_pos_y < -self.phase_tolerance):
            # Takeoff Phase
            self.min_roll = -30
            self.max_roll = 30
            rollObject = PIDCalculation(self.min_roll, self.max_roll, error_pos_y, 'roll', self.i_y, self.prev_e_y,
                                        self.prev_ty)
            roll = rollObject["pidValue"]
            self.i_y = rollObject["i_z"]
            self.prev_e_y = rollObject["prev_e_z"]
            self.prev_ty = rollObject["prev_t"]
            print(f"Roll history: ", self.i_y, self.prev_e_y, self.prev_ty)

            self.min_pitch = -30
            self.max_pitch = 30
            pitchObject = PIDCalculation(self.min_pitch, self.max_pitch, error_pos_x, 'pitch', self.i_x, self.prev_e_x,
                                         self.prev_tx)
            pitch = pitchObject["pidValue"]
            self.i_x = pitchObject["i_z"]
            self.prev_e_x = pitchObject["prev_e_z"]
            self.prev_tx = pitchObject["prev_t"]
            print(f"Pitch history: ", self.i_x, self.prev_e_x, self.prev_tx)

            self.min_throttle = 0
            self.max_throttle = 90
            throttleObject = PIDCalculation(self.min_throttle, self.max_throttle, error_pos_z, 'throttle', self.i_z,
                                            self.prev_e_z, self.prev_t)
            throttle = throttleObject["pidValue"]
            self.i_z = throttleObject["i_z"]
            self.prev_e_z = throttleObject["prev_e_z"]
            self.prev_t = throttleObject["prev_t"]
            print(f"Throttle history: ", self.i_z, self.prev_e_z, self.prev_t)

            print("Requesting control input of:")
            print(f"{throttle=}")
            print(f"{pitch=}")
            print(f"{roll=}")
            # Send control
            control = pb2.DroneClientMsg(
                throttle=int(throttle),
                pitch=int(pitch),
                roll=int(roll),
            )
            self.send_queue.put(control)

        elif (error_pos_x < self.phase_tolerance or error_pos_x > -self.phase_tolerance) and (error_pos_y < self.phase_tolerance or error_pos_y > -self.phase_tolerance) :
            # Stable Phase
            # Apply PID control
            self.min_roll = -8
            self.max_roll = 8
            rollObject = PIDCalculation(self.min_roll, self.max_roll, error_pos_y, 'roll', self.i_y, self.prev_e_y,
                                        self.prev_ty)
            roll = rollObject["pidValue"]
            self.i_y = rollObject["i_z"]
            self.prev_e_y = rollObject["prev_e_z"]
            self.prev_ty = rollObject["prev_t"]
            print(f"Roll history: ", self.i_y, self.prev_e_y, self.prev_ty)

            self.min_pitch = -8
            self.max_pitch = 8
            pitchObject = PIDCalculation(self.min_pitch, self.max_pitch, error_pos_x, 'pitch', self.i_x, self.prev_e_x,
                                         self.prev_tx)
            pitch = pitchObject["pidValue"]
            self.i_x = pitchObject["i_z"]
            self.prev_e_x = pitchObject["prev_e_z"]
            self.prev_tx = pitchObject["prev_t"]
            print(f"Pitch history: ", self.i_x, self.prev_e_x, self.prev_tx)

            self.min_throttle = 40
            self.max_throttle = 70
            throttleObject = PIDCalculation(self.min_throttle, self.max_throttle, error_pos_z, 'throttle', self.i_z,
                                            self.prev_e_z, self.prev_t)
            throttle = throttleObject["pidValue"]
            self.i_z = throttleObject["i_z"]
            self.prev_e_z = throttleObject["prev_e_z"]
            self.prev_t = throttleObject["prev_t"]
            print(f"Throttle history: ", self.i_z, self.prev_e_z, self.prev_t)

            print("Requesting control input of:")
            print(f"{throttle=}")
            print(f"{pitch=}")
            print(f"{roll=}")
            # Send control
            control = pb2.DroneClientMsg(
                throttle=int(throttle),
                pitch=int(pitch),
                roll=int(roll),
            )
            self.send_queue.put(control)

    def receive(self):
        return next(self.event_stream)

    def control_loop(self):
        """Controls the drone until crash, success, or some other failure"""
        while self.simulation_state != SimulationState.ENDED:
            result = self.receive()
            if self.simulation_state == SimulationState.INIT:
                # Check if the first message is simulate start, and save the goal
                if result.WhichOneof("data") == "start":
                    self.goal_region = result.start.goal
                    print("Goal region: ", self.goal_region)
                    self.position = result.start.drone_location
                    self.simulation_state = SimulationState.STARTED
                    self.sim_start_time = time.time() 
                    continue
                else:
                    raise ValueError("Did not receive Simulation Start message as first message")

            # If we receive SimOver ("ended") - end
            if result.WhichOneof("data") == "ended":
                self.simulation_state = SimulationState.ENDED
                total_time = time.time() - self.sim_start_time
                print("Simulation ended. Success: ", result.ended.success, " (", result.ended.details, ")")
                print(f"------ Total time in good zone: {self.time_in_good_zone:.2f}s / {total_time:.2f}s ------")
                return


            # Update - pass into PID loop
            if result.WhichOneof("data") == "update":
                self.position = result.update.drone_location

                if not self.has_taken_off and self.position.z > 0.02:  # 2 cm threshold
                    self.has_taken_off = True

                # time tracking
                now = time.time()
                if self.last_update_time is None:
                    self.last_update_time = now
                dt = now - self.last_update_time
                self.last_update_time = now

                # check if inside good zone
                if (self.goal_region.minimal_point.x <= self.position.x <= self.goal_region.maximal_point.x and
                    self.goal_region.minimal_point.y <= self.position.y <= self.goal_region.maximal_point.y and
                    self.goal_region.minimal_point.z <= self.position.z <= self.goal_region.maximal_point.z):
                    self.time_in_good_zone += dt

                print(f"Drone has spent {self.time_in_good_zone:.2f} seconds in the good zone")

                # check for crash (drone hits ground)
                if self.has_taken_off and self.position.z <= 0.01:
                    self.simulation_state = SimulationState.ENDED
                    total_time = time.time() - self.sim_start_time
                    print("Simulation ended: Drone hit the ground")
                    percentage = (self.time_in_good_zone / total_time) * 100 if total_time > 0 else 0
                    print(f"\n------ Total time in good zone: {self.time_in_good_zone:.2f}s / {total_time:.2f}s ({percentage:.1f}%) ------\n")

                    return

                # Run control
                self.control_drone()


def load_token():
    path = os.getenv("DRONE_TOKEN_FILE", "token.txt")
    try:
        with open(path, "r") as f:
            for line in f:
                line = line.strip()
                if line:
                    return line
    except FileNotFoundError:
        pass
    raise ValueError("No token provided! Set DRONE_TOKEN env or put it in token.txt")

if __name__ == "__main__":
    host = "172.104.137.51"
    port = 10301
    # TODO: add your token here
    token = load_token()
    if token is None:
        raise ValueError("No token provided!")
    dc = DroneClient(host, port, token)
    dc.start()
