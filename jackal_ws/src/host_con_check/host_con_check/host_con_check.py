import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from jackal_msgs.msg import StopStatus
import socket
import os
import time

class HostConCheck(Node):

    def __init__(self):
        super().__init__('host_con_check')
        self.publisher_ = self.create_publisher(Bool, 'e_stop', 10) #create a publisher for the /e_stop
        self.stop_status_sub = self.create_subscription(StopStatus, "stopstatus", self.stop_status_callback,10)
        self.last_changed_state = False
        self.stop_status = False
        
        #host setting
        self.ip = "10.42.0.130"
        self.port = 443
        self.retry = 2
        self.timeout = 1
        self.delay = 0.2
        
        #Set a timer for the check host
        timer_period = (self.timeout + self.delay)*self.retry + 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def timer_callback(self):
        msg = Bool()
        connection_state = self.check_host(self.ip, self.port)
        msg.data = not connection_state
        if (self.last_changed_state == connection_state):
            if not ((self.last_changed_state != self.stop_status) and (self.stop_status == True)): 
                #when the last changed state is not same with the stop status and the stop status is True which means the host may enable the e-stop. Don't change it.
                self.publisher_.publish(msg)
                self.get_logger().info('Publishing: "%s"' % msg.data)
                self.last_changed_state = msg.data
        else:
            pass
        
        
    def is_open(self,ip, port):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(self.timeout)
        try:
                #s.connect((ip, int(port)))
                #s.shutdown(socket.SHUT_RDWR)
                
                response = os.system(f"ping -c 1 {self.ip}")
                if response == 0:
                    return True
                else:
                    return False
        except:
                return False
        #finally:
                #s.close()
                
    def check_host(self, ip, port):
        ip_up = False
        for i in range(self.retry):
                if self.is_open(ip, port):
                        ip_up = True
                        break
                else:
                        time.sleep(self.delay)
        return ip_up
    
    def stop_status_callback(self,msg):

        self.stop_status = msg._stop_power_status
        self.get_logger().info('Publishing: "%s"' % msg._stop_power_status)
    






def main():
    rclpy.init()

    host_con_check = HostConCheck()

    rclpy.spin(host_con_check)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    host_con_check.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
