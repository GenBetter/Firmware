#以下是仿照manual_control_setpoint.msg编写的，包括注释也没有修改，目的就是提醒自己 注意两个遥控器的区别，正确使用别出错，如z油门摇杆的范围是不同的。
#原来的遥控器数据实在px4io.cpp中获取的 主题名字叫input_rc
#而后在sensor.cpp中进行归一化，归一化到-1到1，归一化后的主题名字叫manual_control_setpoint
#而后manual_control_setpoint这个主题传递到位置控制和姿态控制中进行使用
#最后处理产生actuator_control.msg伺服器指令，结合混空气产生pwm输出

uint8[12] channel #遥控器11个通道的原始数据，预留一个字节备用 同时也保证内存字节的对齐

float32 x			 # stick position in x direction -1..1
				 # in general corresponds to forward/back motion or pitch of vehicle,
				 # in general a positive value means forward or negative pitch and
				 # a negative value means backward or positive pitch
float32 y			 # stick position in y direction -1..1
				 # in general corresponds to right/left motion or roll of vehicle,
				 # in general a positive value means right or positive roll and
				 # a negative value means left or negative roll
float32 z			 # throttle stick position 0..1
				 # in general corresponds to up/down motion or thrust of vehicle,
				 # in general the value corresponds to the demanded throttle by the user,
				 # if the input is used for setting the setpoint of a vertical position
				 # controller any value > 0.5 means up and any value < 0.5 means down
float32 r			 # yaw stick/twist position, -1..1
				 # in general corresponds to the righthand rotation around the vertical
				 # (downwards) axis of the vehicle




