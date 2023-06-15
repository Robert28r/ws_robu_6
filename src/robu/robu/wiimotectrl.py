import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from rclpy.node import Node

import cwiid 

#define CWIID_BTN_2		0x0001
#define CWIID_BTN_1		0x0002
#define CWIID_BTN_B		0x0004
#define CWIID_BTN_A		0x0008
#define CWIID_BTN_MINUS	0x0010
#define CWIID_BTN_HOME	0x0080
#define CWIID_BTN_LEFT	0x0100
#define CWIID_BTN_RIGHT	0x0200
#define CWIID_BTN_DOWN	0x0400
#define CWIID_BTN_UP	0x0800
#define CWIID_BTN_PLUS	0x1000


class WiimoteCtrl(Node):
  def __init__(self, default_lin_vel = 0.05, default_ang_vel = 0.25):
    super().__init__('WiimoteCtrl')

    self._default_lin_vel = default_lin_vel
    self._default_ang_vel = default_ang_vel

    if self.connect() is None:
      return

    #set Wiimote to report button presses and accelerometer state 
    self.wm.rpt_mode = cwiid.RPT_BTN | cwiid.RPT_ACC 
    self.wm_mode = cwiid.RPT_BTN | cwiid.RPT_ACC

    qos = QoSProfile(depth=10)
    self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', qos)
    
    self.msg_cmd_vel = None

    self.update_timer = self.create_timer(
        0.100,  # unit: s
        self.timer_callback)

    self.get_logger().info("WiimoteCtrl has been initialised.")

  def connect(self):
    #connecting to the Wiimote. This allows several attempts 
    # as first few often fail. 
    msg_error = "Error opening wiimote connection"
    print ('Press 1+2 on your Wiimote now...') 
    wm = None 
    i=2 
    while not wm: 
      try: 
        wm=cwiid.Wiimote() 
      except RuntimeError: 
        if (i>10): 
          break 
        print (msg_error) 
        print ("attempt " + str(i)) 
        i +=1 
    
    if wm is not None:
      #turn on led to show connected 
      wm.led = 1
    else:
      self.get_logger().error(msg_error)

    self.wm = wm
    return wm
  
  def timer_callback(self):
    msg_cmd_vel = Twist()
    msg_cmd_vel.linear.y = 0.0
    msg_cmd_vel.linear.z = 0.0
    msg_cmd_vel.angular.x = 0.0
    msg_cmd_vel.angular.y = 0.0

    if (self.wm is not None):
      if (self.wm_mode & cwiid.RPT_BTN):
        btn_state = self.wm.state['buttons']
        msg_cmd_vel.linear.x = 0.0
        msg_cmd_vel.angular.z = 0.0

        if btn_state & cwiid.BTN_RIGHT:
          msg_cmd_vel.linear.x = self._default_lin_vel
      
        if btn_state & cwiid.BTN_LEFT:
          msg_cmd_vel.linear.x = -self._default_lin_vel

        if btn_state & cwiid.BTN_DOWN:
          msg_cmd_vel.angular.z = -self._default_ang_vel

        if btn_state & cwiid.BTN_UP:
          msg_cmd_vel.angular.z = self._default_ang_vel

        if btn_state & cwiid.BTN_2:
          if (btn_state & cwiid.BTN_RIGHT) or (btn_state & cwiid.BTN_LEFT):
            msg_cmd_vel.linear.x = 4 * msg_cmd_vel.linear.x
          else:
            msg_cmd_vel.angular.z = 4 * msg_cmd_vel.angular.z

        if ( (self.msg_cmd_vel is None) or (round(msg_cmd_vel.linear.x,2) != round(self.msg_cmd_vel.linear.x,2)) or (round(msg_cmd_vel.angular.z,2) != round(self.msg_cmd_vel.angular.z,2))):
          self.get_logger().info("Publish lin.x: " + str(msg_cmd_vel.linear.x) +" ang.z: " + str(msg_cmd_vel.angular.z))
          self.pub_cmd_vel.publish(msg_cmd_vel)
          self.msg_cmd_vel = msg_cmd_vel

      elif (self.wm_mode & cwiid.RPT_ACC):
        pass

def main(args=None):
  rclpy.init(args=args)

  wiimotectrl_node = WiimoteCtrl()

  rclpy.spin(wiimotectrl_node)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  wiimotectrl_node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
    main()