#!/usr/bin/env python
# encoding: utf-8

import rclpy
from rclpy.node import Node
import sys, select, termios, tty
from geometry_msgs.msg import Twist

# 控制提示
tip = """
Carbot Key Ctrl
--------------------------------------------------
x: w/x	y: 1/2	z: a/d
space: force stop
--------------------------------------------------
q/z : increase/decrease all speeds
--------------------------------------------------
CTRL-C to quit
"""

moveBindings = {'w': (1, 0, 0),
                '1': (0, 1, 0),
                'x': (-1, 0, 0),
                '2': (0, -1, 0),
                'a': (0, 0, 1),
                'd': (0, 0, -1),

                'W': (1, 0, 0),
                'X': (-1, 0, 0),
                'A': (0, 0, 1),
                'D': (0, 0, -1),}

speedBindings = {'q': (1.3, 1.3),
                 'z': (0.7, 0.7),
				 'Q': (1.3, 1.3),
                 'Z': (0.7, 0.7),
				 
				 }

class Key_Ctrl(Node):
	# 节点初始化
	def __init__(self,name):
		super().__init__(name)
        # 创建发布者 
		self.ctrl_pub = self.create_publisher(Twist,'turtle1/cmd_vel',2)
  
        # 参数声明
		self.declare_parameter("linear_speed_limit",2.0)
		self.declare_parameter("angular_speed_limit",2.0)
		self.declare_parameter("auto_brake",False)
		self.linenar_speed_limit = self.get_parameter("linear_speed_limit").get_parameter_value().double_value
		self.angular_speed_limit = self.get_parameter("angular_speed_limit").get_parameter_value().double_value
		self.auto_brake = self.get_parameter("auto_brake").get_parameter_value().bool_value
  
		# tcgetattr函数用于获取与终端相关的参数
		self.settings = termios.tcgetattr(sys.stdin)

	# 监测按键输入
	def getKey(self):
		tty.setraw(sys.stdin.fileno())
		rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
		if rlist: key = sys.stdin.read(1)
		else: 
			key = ''
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
		return key

	def vels(self, speed, turn):
		return "currently:\tspeed %s\tturn %s " % (speed,turn)		
	
def main():
	rclpy.init()
	key_ctrl = Key_Ctrl("key_ctrl")
	# 速度初始化
	(speed, turn) = (0.2, 0.15)
	(x_linear, y_linear, z_angular) = (0, 0, 0)
	twist = Twist()
	try:
		print(tip)
		while True:
			# 键盘监听
			key = key_ctrl.getKey()
			if(key):
				if key in moveBindings.keys():
					x_linear = moveBindings[key][0]
					y_linear = moveBindings[key][1]
					z_angular = moveBindings[key][2]
				elif key in speedBindings.keys():
					speed = speed * speedBindings[key][0]
					turn = turn * speedBindings[key][1]
					# 速度上限监测
					if speed > key_ctrl.linenar_speed_limit: 
						speed = key_ctrl.linenar_speed_limit
					if turn > key_ctrl.angular_speed_limit: 
						turn = key_ctrl.angular_speed_limit
					print("linear speed: %f\tangular speed: %f" % (speed, turn))
					continue
				elif key == '\x03':
					break
				else:
					if key_ctrl.auto_brake:
						twist = Twist()
					else:
						pass

				twist.linear.x = speed * x_linear
				twist.linear.y = speed * y_linear
				twist.angular.z = turn * z_angular

				key_ctrl.ctrl_pub.publish(twist)
			else:
				twist.linear.x = 0.0
				twist.linear.y = 0.0
				twist.angular.z = 0.0
				key_ctrl.ctrl_pub.publish(twist)

			

	except Exception as e: 
		print(e)

	finally: 
		key_ctrl.ctrl_pub.publish(Twist())

	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, key_ctrl.settings)
	key_ctrl.destroy_node()
	rclpy.shutdown()