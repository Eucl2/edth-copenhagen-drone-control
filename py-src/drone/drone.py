import queue
from enum import Enum

import generated.drone_pb2 as pb2
import generated.drone_pb2_grpc as pb2_gprc
import grpc
import time



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

        big_min_target_pos_x = min_target_pos.x * 2
        big_min_target_pos_y = min_target_pos.y * 2
        big_max_target_pos_x = max_target_pos.x * 2
        big_max_target_pos_y = max_target_pos.y * 2

        throttle = 75
        pitch = 0
        roll = 0

        #find center of target region
        x_best = (min_target_pos.x + max_target_pos.x) / 2
        y_best = (min_target_pos.y + max_target_pos.y) / 2
        z_best = (min_target_pos.z + max_target_pos.z) / 2

        # adjust speed based on distance to center
        dx = x_best - self.position.x
        dy = y_best - self.position.y

        # slow down if inside big square
        if big_min_target_pos_x <= self.position.x <= big_max_target_pos_x and \
        big_min_target_pos_y <= self.position.y <= big_max_target_pos_y:
            pitch = max(-2, min(2, dx))
            roll = max(-2, min(2, dy))
        else:
            pitch = dx
            roll = dy

        # altitude
        if self.position.z >= z_best:
            throttle = 0
        else:
            throttle = 71


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



if __name__ == "__main__":
    host = "172.104.137.51"
    port = 10301
    # TODO: add your token here
    token = "b794ce6c-8b8c-4330-9c4c-85089892940c"
    if token is None:
        raise ValueError("No token provided!")
    dc = DroneClient(host, port, token)
    dc.start()
