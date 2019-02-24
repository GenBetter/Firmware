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




#因为要用遥控器进行解锁 上锁等操作，所以最终遥控器的数据还是以manual_control_setpoint这个主题数据发布为主
#但需要注意 guanghua_rc目前发布了但是没使用

#使用manual_control_setpoint需要找出原来主题使用的所有的地方 看看有没有影响
#使用manual_control_setpoint需要屏蔽原来的发布 变成telem2.c中发布光华的遥控器数据
#注意重要的z油门数据的范围是不一样的
#虽然z轴 正常的遥控器范围是0-1，光华的是-1到1，但我发现在真实的使用过程中
#在定高模式的使用过程中 油门还是会归一化到-1到1的，意思就是光华的遥控器可以正常使用，把使用范围初始化0-1 和正常的遥控器保持一致
#这样在定高模式时 光华的遥控器自动回中的功能 正是定高模式所需要的

#简单来说光华的遥控器可以看做普通遥控器来用，z轴范围视为0-1（最下面0 最上面1 默认中间0.5），不过当做普通遥控器来用的前提是飞机只能飞在定高模式
#所有摇杆的数据不控姿态 控的是速度，遥控数据实在位置控制中control_manual函数里处理的
