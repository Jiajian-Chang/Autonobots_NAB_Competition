import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import socket
import time

class HostConCheck(Node):

    def __init__(self):
        super().__init__('host_con_check')
        self.publisher_ = self.create_publisher(Bool, 'e_stop', 10) #create a publisher for the /e_stop
        self.last_check_state = False
        
        #host setting
        self.ip = "google.com"
        self.port = 443
        self.retry = 2
        self.timeout = 1
        self.delay = 0.2
        
        #Set a timer for the check host
        timer_period = (self.timeout + self.delay)*self.retry + 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def timer_callback(self):
        msg = Bool()
        connection_state = self.checkHost(self.ip, self.port)
        msg.data = not connection_state
        if self.last_check_state != connection_state:
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
            self.last_check_state = connection_state
        else:
            pass
        
        
    def isOpen(self,ip, port):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(self.timeout)
        try:
                s.connect((ip, int(port)))
                s.shutdown(socket.SHUT_RDWR)
                return True
        except:
                return False
        finally:
                s.close()
                
    def checkHost(self, ip, port):
        ipup = False
        for i in range(self.retry):
                if self.isOpen(ip, port):
                        ipup = True
                        break
                else:
                        time.sleep(self.delay)
        return ipup





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
