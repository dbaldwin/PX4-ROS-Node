import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, VehicleStatus, VehicleGlobalPosition
from std_msgs.msg import String

from pysm import State, StateMachine, Event
from queue import Queue

class PX4Demo(Node):

    def __init__(self):
        ############################ R O S  S E T U P #################################
        super().__init__(node_name='px4_demo') #type: ignore
        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.vehicle_status_callback, qos_profile)
        self.vehicle_global_pos_subscriber = self.create_subscription(VehicleGlobalPosition, "/fmu/out/vehicle_global_position", self.vehicle_global_pos_callback, qos_profile)
        
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos_profile)

        exec_frequency = 20 # Hz
        timer_period = 1/exec_frequency  # seconds
        self.frame_timer = self.create_timer(timer_period, self.exec_frame)
        ###############################################################################

        #################### P X 4  S T A T E  S T O R A G E ##########################
        self.prev_vehicle_status_msg = None
        self.lat = 0.00
        self.lon = 0.00
        self.alt = 0.00
        ###############################################################################

        #################### S T A T E  M A C H I N E  S E T U P ######################
        # event queue
        self.event_queue = Queue()

        # # message in queues
        # self.vehicle_status_queue = Queue()
        # self.vehicle_global_pos_queue = Queue()

        # # message out queues
        # self.msg_out_queue = Queue()

        # create the init state
        self.init_state = State('init')
        
        # create the Disarm state
        self.disarm_state = State('Disarm')
        self.disarm_state.handlers = {
            'enter': self.disarm_enter,
        }
        
        # create the Arm state
        self.arm_state = State('Arm')
        self.arm_state.handlers = {
            'enter': self.arm_enter,
        }
        
        # create the Takeoff state
        self.takeoff_state = State('Takeoff')
        
        # create the Land state
        self.landing_state = State('Land')

        # create the state machine
        self.sm = StateMachine('sm')
        
        # add the states
        self.sm.add_state(self.init_state, initial=True)
        self.sm.add_state(self.disarm_state)
        self.sm.add_state(self.arm_state)
        self.sm.add_state(self.takeoff_state)
        self.sm.add_state(self.landing_state)
        
        # add the transitions
        self.sm.add_transition(self.init_state, self.disarm_state, events=['vehicle_status_init_event'])
        self.sm.add_transition(self.disarm_state, self.arm_state, events=['vehicle_armed_event'])
        self.sm.add_transition(self.arm_state, self.takeoff_state, events=['vehicle_takeoff_event'])
        self.sm.add_transition(self.takeoff_state, self.landing_state, events=['vehicle_begin_landing_event'])
        self.sm.add_transition(self.landing_state, self.disarm_state, events=['vehicle_disarmed_event'])

        self.sm.initialize()
        ###############################################################################

        
    #################### S T A T E  M A C H I N E  M E T H O D S ######################
    def disarm_enter(self, state, event):
        self.arm()

    def arm_enter(self, state, event):
        self.takeoff(5)
    ####################################################################################

    ########################### R O S  N O D E  M E T H O D S ##########################
    def exec_frame(self):
        while not self.event_queue.empty():
            event = self.event_queue.get()
            self.get_logger().info(f"Got a {event.name}")
            old = self.sm.state.name # type: ignore
            self.sm.dispatch(event)
            self.get_logger().info(f"TRANSITIONED FROM: ({old}) ---> ({self.sm.state.name})") # type: ignore
    
    def vehicle_status_callback(self, msg: VehicleStatus):
        
        if self.prev_vehicle_status_msg == None:
            self.event_queue.put(Event("vehicle_status_init_event"))
        else:
            #check arming
            if self.prev_vehicle_status_msg.arming_state != msg.arming_state:
                if msg.arming_state == VehicleStatus.ARMING_STATE_STANDBY: #disarmed
                    self.event_queue.put(Event("vehicle_disarmed_event"))
                elif msg.arming_state == VehicleStatus.ARMING_STATE_ARMED: # armed
                    self.event_queue.put(Event("vehicle_armed_event"))
            #check nav_state
            if self.prev_vehicle_status_msg.nav_state != msg.nav_state:
                if msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF: # takeoff
                    self.event_queue.put(Event("vehicle_takeoff_event"))
                elif msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND: # land
                    self.event_queue.put(Event("vehicle_begin_landing_event"))
        # self.get_logger().info("GOT A MESSAGE")
        self.prev_vehicle_status_msg = msg
    
    def vehicle_global_pos_callback(self, msg: VehicleGlobalPosition):
        self.lat = msg.lat
        self.lon = msg.lon
        self.alt = msg.alt
        # self.get_logger().info(f"Lat: {self.lat} Lon: {self.lon} Alt: {self.alt}")
    ###################################################################################

    ########################### P X 4  R O S  M E T H O D S ###########################
    def send_vehicle_command(self, msg: VehicleCommand):
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # if the msg has a bunch of defaults set, set them to our defaults
        if all([
                msg.target_system == 0,
                msg.target_component == 0,
                msg.source_system == 0,
                msg.source_component == 0
                ]):
            msg.target_system = 1
            msg.target_component = 1
            msg.source_system = 1
            msg.source_component = 1
            
        msg.from_external = True
        
        self.vehicle_command_publisher.publish(msg)
        self.get_logger().info(f'Sent command {msg.command}')

    def arm(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0 # arm (0 disarm 1 arm)
        msg.param2 = float(0) # force (0 for no 21196 for force)
        self.send_vehicle_command(msg)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 0.0 # arm (0 disarm 1 arm)
        msg.param2 = float(0) # force (0 for no 21196 for force)
        self.send_vehicle_command(msg)
        self.get_logger().info('Disarm command sent')

    def takeoff(self, alt):
        '''
        take off to the specified MSL altitude
        '''
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
        msg.param1 = float(-1)
        msg.param2 = float(0)
        msg.param3 = float(0)
        msg.param4 = float('nan')
        msg.param5 = float('nan')
        msg.param6 = float('nan')
        msg.param7 = float(alt + self.alt) 

        self.send_vehicle_command(msg)

        self.get_logger().info('Takeoff command sent')
    
    def go_to_location(self, lat, lon, alt_msl, ground_speed = 5):
        '''
        Reposition the vehicle to a specific WGS84 global position.
        '''
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_REPOSITION
        msg.param1 = float(ground_speed) # ground speed
        msg.param2 = 1.0 # bitmask
        msg.param3 = float('nan') # loiter radius for planes
        msg.param4 = float('nan') # yaw heading
        msg.param5 = float(lat) # lat
        msg.param6 = float(lon) # lon
        msg.param7 = float(alt_msl)# alt

        self.vehicle_command_publisher.publish(msg)

        self.get_logger().info('Reposition command sent')

    def land(self):
        '''
        switch to land mode
        '''
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        self.send_vehicle_command(msg)
        self.get_logger().info('Land command sent')
    ###################################################################################
    

def main(args=None):
    rclpy.init(args=args)
    try:
        px4_demo = PX4Demo()
        rclpy.spin(px4_demo)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        px4_demo.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

if __name__ == '__main__':
    main()
