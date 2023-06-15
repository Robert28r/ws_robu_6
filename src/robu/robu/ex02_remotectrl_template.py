#Exercise Title:    Remote Control for the TurtleBot3 Burger
#Group:             ?
#Class:             ?
#Date:              ?

import rclpy

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
import msvcrt


msg = """
Excercise:  ?
Group:      ?
Class:      ?
Date:       ?
"""

e = """
Communications Failed
"""

"""
72... Cursor Beschleunigen
80... Cursor Bremsen
75... Cursor Drehung nach links
77... Cursor Drehung nach rechts
"""
key_ctrl = [72, 80, 75, 77]


#Physikalische Grenzen des Roboters
MAX_LIN_VEL = 0.22          #m/s
MAX_ANG_VEL = 2.84          #rad/s

LIN_VEL_STEP_SIZE = 0.01    #m/s
ANG_VEL_STEP_SIZE = 0.1     #rad/s

def get_key():
    try:
        return msvcrt.getch().decode('cp1252') #.decode('utf-8')
    except:
        return msvcrt.getch().decode('utf-8')

        
def main():
    
    key_null_entered = False

    rclpy.init()

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('remotectrl')
    pub = node.create_publisher(Twist, 'cmd_vel', qos)

    vel = Twist()

    try:
        while(1):
            key = get_key()
            print("Key: ", key, "ASCII: ", ord(key))

            if ord(key) == 0x00:
                key_null_entered = True

            if key_null_entered == True and key_ctrl[0] == ord(key): #Beschleunigen
                vel.linear.x
            elif key_null_entered == True and key_ctrl[1] == ord(key): # Bremsen
                pass
            
            pub.publish(vel)

    except Exception as e:
        print(e)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
