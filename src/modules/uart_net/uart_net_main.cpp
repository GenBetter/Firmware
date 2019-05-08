#include <termios.h>
#include <drivers/drv_hrt.h>
#include <systemlib/mavlink_log.h>
#include <px4_posix.h>
#include <vector>
#include <deque>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <poll.h>

static orb_advert_t mavlink_log_pub = 0;
static bool thread_running = false;
static bool thread_should_exit = false;
static int daemon_task;

int uart_net_thread(int argc, char *argv[]);
extern "C" __EXPORT int uart_net_main(int argc, char *argv[]);

int uart_net_main(int argc, char *argv[])
{
    if (argc < 2) {
        mavlink_log_critical(&mavlink_log_pub, "[uart_net]mission command");
    }

    if (!strcmp(argv[1], "start")) {
        if (thread_running) {
            mavlink_log_critical(&mavlink_log_pub, "[uart_net]already running");
            exit(0);
        }
        
        thread_should_exit = false;
        thread_running = true;
        daemon_task = px4_task_spawn_cmd("uart_net",
                                         SCHED_DEFAULT,
                                         SCHED_PRIORITY_MAX - 5,
                                         3000,
                                         uart_net_thread,
                                         &argv[2]);

        return 0;
        exit(0);
    }
    
    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        exit(0);
    }
    
    mavlink_log_critical(&mavlink_log_pub, "unrecognized command");
    exit(1);
    return 0;
}

int uart_net_thread(int argc, char *argv[])
{
    home_position_s _home_pos;
    int home_pos_sub = orb_subscribe(ORB_ID(home_position));
    px4_pollfd_struct_t fds;
    fds.fd = home_pos_sub;
    fds.events = POLLIN;
    while(1)
    {
        int pret = px4_poll(&fds, 1, 1000);//等待home_position位置
        if (pret <= 0)
        {
            mavlink_log_critical(&mavlink_log_pub, "recv _home_pos over 1 second,continue!");
            continue;//如果超时继续获取直到获取成功
        }

        orb_copy(ORB_ID(home_position), home_pos_sub, &_home_pos);

        break;
    }
    mavlink_log_critical(&mavlink_log_pub, "home pos getted!");//这里表示home点已经获取到了

    //这里开始获取vehicle_status,有vehicle_status_s里面的system_id和component_id才能发送vehicle_command命令飞机
    vehicle_status_s _status;
    int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
    fds.fd = vehicle_status_sub;
    while(1)//这个循环一直等待直到获取成功
    {
        int pret = px4_poll(&fds, 1, 1000);
        if (pret <= 0)
        {
            mavlink_log_critical(&mavlink_log_pub, "recv _status over 1 second,continue!");
            continue;
        }

        orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &_status);
       
       // printf("nav_state = %d\n",_status.nav_state);
       //选择一种模式触发offboard 选择land模式下触发offboard,这样offboard结束后可以直接返回到land模式执行降落
        if(_status.nav_state==2)// NAVIGATION_STATE_AUTO_LAND = 18;;NAVIGATION_STATE_POSCTL=2定点模式 从定点模式切进offboard
        {
            break;
        }
        else{

        }
        
    }
    mavlink_log_critical(&mavlink_log_pub, "vehicle_status_s getted!");
    //这里表示获取成功了

    orb_advert_t vehicle_command_pub = nullptr;
    vehicle_command_s _command = {};
    _command.target_system = _status.system_id;//system_id写入
    _command.target_component = _status.component_id;//component_id写入
    int vehicle_command_ack_sub = orb_subscribe(ORB_ID(vehicle_command_ack));//订阅vehicle_command_ack
    //vehicle_command_ack是发送命令vehicle_command的返回信息,可以查看命令是否执行成功

    //offboard_control_mode topic的初始化 这个主题必须以每秒2次的频率publish才能让飞机一直维持offboard模式
    offboard_control_mode_s ocm;
    memset(&ocm, 0, sizeof(offboard_control_mode_s));
    ocm.ignore_acceleration_force = true;//这个很重要,必须要强制关闭加速度控制才能进行速度控制,位置控制
    orb_advert_t offboard_pub = nullptr;

    //从这里开始就是offboard模式的位置控制了,要飞到哪个点，只需要填入相应的xyz，单位是米，坐标系是ned
    orb_advert_t _pos_sp_triplet_pub = nullptr;
    position_setpoint_triplet_s _pos_sp_triplet;
    memset(&_pos_sp_triplet, 0, sizeof(position_setpoint_triplet_s));//全部清0
    _pos_sp_triplet.timestamp = hrt_absolute_time();//写当前时间，非必须
    _pos_sp_triplet.current.valid = true;//让3组合的current为可用,且pre和next必须为false,因为位置控制模块的offboard模式没有用到这两个
    _pos_sp_triplet.current.position_valid = true;//这里我们只控制位置,不控制速度、yaw角和加速度 所以只需要开启这个标识位
    _pos_sp_triplet.current.x = 0.0f;
    _pos_sp_triplet.current.y = 0.0f;
    _pos_sp_triplet.current.z = -1.5f;//一开始我们飞到home点上方3米高空悬停
    _pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
    //要移动飞机，这个type必须是position_setpoint_s::SETPOINT_TYPE_POSITION，其他还有这些type
//    if (is_takeoff_sp) {
//        pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_TAKEOFF;
    //这个表示起飞，其实可以不用，直接用position就可以起飞

//    } else if (is_land_sp) {
//        pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LAND;//着陆

//    } else if (is_loiter_sp) {
//        pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;//悬停

//    } else if (is_idle_sp) {
//        pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_IDLE;//直接让马达速度变成怠速，自己做降落的时候可以使用

//    } else {
//        pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;//这个最关键，我们最常用的
//    }

    uint64_t timetick = hrt_absolute_time();
    int sended = 0;

	
    //这个循环控制让飞机开机的时候先切进postion control mode
   while(1)
    {
        //发送vehicle_command
        _command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;//设置模式命令
        _command.param1 = 213;//主模式为costom
        _command.param2 = 4;//二级模式为position control
        _command.param3 = 6;//三级模式没有！
        if (vehicle_command_pub != nullptr) {
            orb_publish(ORB_ID(vehicle_command), vehicle_command_pub, &_command);
        } else {
            vehicle_command_pub = orb_advertise(ORB_ID(vehicle_command), &_command);
        }

        vehicle_command_ack_s _ack;
        fds.fd = vehicle_command_ack_sub;
        //这里循环接受vehicle_command的返回值vehicle_command_ack
        while(1)
        {
            int pret = px4_poll(&fds, 1, 1000);
            if (pret <= 0)
            {
                mavlink_log_critical(&mavlink_log_pub, "recv _ack over 1 second,continue!");
                continue;
            }

            orb_copy(ORB_ID(vehicle_command_ack), vehicle_command_ack_sub, &_ack);

            break;
        }

        if (_ack.result == vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED)//如果返回值是这个表示切换成功
        {
            break;
        }

        //如果没有跳出表示切换失败,循环回去再切换
        mavlink_log_critical(&mavlink_log_pub, "can't go into posctl mode,continue!");
        usleep(10000);
    }
    mavlink_log_critical(&mavlink_log_pub, "land mode ok!");
	
	

    while(1)
    {
        ocm.timestamp = hrt_absolute_time();
        if (offboard_pub != nullptr) {
            orb_publish(ORB_ID(offboard_control_mode), offboard_pub, &ocm);
        } else {
            offboard_pub = orb_advertise(ORB_ID(offboard_control_mode), &ocm);
        }

        _command.command = vehicle_command_s::VEHICLE_CMD_NAV_GUIDED_ENABLE;//这个指令就是进入offboard模式的指令
        //但是在这之前，我们需要先发布一次offboard_control_mode，否则无法进入offboard，常见上面的代码
        _command.param1 = 1.0f;
        if (vehicle_command_pub != nullptr) {
            orb_publish(ORB_ID(vehicle_command), vehicle_command_pub, &_command);
        } else {
            vehicle_command_pub = orb_advertise(ORB_ID(vehicle_command), &_command);
        }

        //这里取返回值，跟上面的代码一样
        vehicle_command_ack_s _ack;
        fds.fd = vehicle_command_ack_sub;
        while(1)
        {
            int pret = px4_poll(&fds, 1, 1000);
            if (pret <= 0)
            {
                mavlink_log_critical(&mavlink_log_pub, "recv _ack over 1 second,continue!");
                continue;
            }

            orb_copy(ORB_ID(vehicle_command_ack), vehicle_command_ack_sub, &_ack);

            break;
        }

        if (_ack.result == vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED)
        {
            break;
        }

        //如果进入失败，继续循环重试
        mavlink_log_critical(&mavlink_log_pub, "can't go into offboard mode,continue!");
        usleep(10000);
    }
    mavlink_log_critical(&mavlink_log_pub, "-- offboard mode ok!");

    //这里开始就已经进入了offboard了 然后我们就可以命令飞机实时飞行了，如果有4g模块就可以做到实时的指点飞行
    //当然下面的代码我只是做了一个小测试，飞了4个航点 左边是写死在代码里的 以后只需要修改坐标的来源为网络就可以
    //用手机 平板进行实时控制
    while(1)
    {
        if(sended == 0)//第一步，解锁 （arm）
        {
            _command.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;//发送指令arm
            _command.param1 = 1.0f;//1.0为解锁 0.0为加锁
            if (vehicle_command_pub != nullptr) {//发布消息
                orb_publish(ORB_ID(vehicle_command), vehicle_command_pub, &_command);
            } else {
                vehicle_command_pub = orb_advertise(ORB_ID(vehicle_command), &_command);
            }

            mavlink_log_critical(&mavlink_log_pub, "-- arm ok!");
            sended++;//进行第二部，这里只是测试代码 我就没有收取命令返回值了 以后做项目的时候记得加上
        }

        if(sended == 1 && hrt_absolute_time() - timetick > 3000000)//3秒以后进行第二步起飞
        {
            sended++;

            //因为前面已经写了初始化的值  所以这里我们直接发布就行，飞机会在home点正上方3米高度悬停
            if (_pos_sp_triplet_pub == nullptr) {
                mavlink_log_critical(&mavlink_log_pub, "-- takeoff");
                _pos_sp_triplet_pub = orb_advertise(ORB_ID(position_setpoint_triplet), &_pos_sp_triplet);
            } else {
                mavlink_log_critical(&mavlink_log_pub, "-- takeoff");
                orb_publish(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_pub, &_pos_sp_triplet);
                timetick = hrt_absolute_time();
            }
        }

        if(sended == 2 && hrt_absolute_time() - timetick > 10000000)//10秒以后进行第一个航点
        {
            sended++;

            //飞到这个坐标
            _pos_sp_triplet.current.x=5.0f;
            _pos_sp_triplet.current.y=5.0f;
            _pos_sp_triplet.current.z=-1.5f;
            mavlink_log_critical(&mavlink_log_pub, "-- position -- 1");
            orb_publish(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_pub, &_pos_sp_triplet);
            timetick = hrt_absolute_time();
        }

        if(sended == 3 && hrt_absolute_time() - timetick > 10000000)//10秒以后进行第二个航点
        {
            sended++;

            _pos_sp_triplet.current.x=5.0f;
            _pos_sp_triplet.current.y=-5.0f;
            _pos_sp_triplet.current.z=-1.5f;
            mavlink_log_critical(&mavlink_log_pub, "-- position -- 2");
            orb_publish(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_pub, &_pos_sp_triplet);
            timetick = hrt_absolute_time();
        }

        if(sended == 4 && hrt_absolute_time() - timetick > 10000000)//10秒以后进行第三个航点
        {
            sended++;

            _pos_sp_triplet.current.x=-5.0f;
            _pos_sp_triplet.current.y=-5.0f;
            _pos_sp_triplet.current.z=-1.5f;
            mavlink_log_critical(&mavlink_log_pub, "-- position -- 3");
            orb_publish(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_pub, &_pos_sp_triplet);
            timetick = hrt_absolute_time();
        }
/*
        if(sended == 5 && hrt_absolute_time() - timetick > 10000000)//10秒以后进行第四个航点
        {
            sended++;

            _pos_sp_triplet.current.x=-5.0f;
            _pos_sp_triplet.current.y=5.0f;
            _pos_sp_triplet.current.z=-1.5f;
            mavlink_log_critical(&mavlink_log_pub, "-- position -- 4");
            orb_publish(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_pub, &_pos_sp_triplet);
            timetick = hrt_absolute_time();
        }

        if(sended == 6 && hrt_absolute_time() - timetick > 10000000)//10秒以后进行第四个航点
        {
            sended++;

            _pos_sp_triplet.current.x=5.0f;
            _pos_sp_triplet.current.y=5.0f;
            _pos_sp_triplet.current.z=-1.5f;
            mavlink_log_critical(&mavlink_log_pub, "-- position -- 5");
            orb_publish(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_pub, &_pos_sp_triplet);
            timetick = hrt_absolute_time();
        }

        if(sended == 7 && hrt_absolute_time() - timetick > 10000000)//10秒以后进行第四个航点
        {
            sended++;

            _pos_sp_triplet.current.x=0.0f;
            _pos_sp_triplet.current.y=0.0f;
            _pos_sp_triplet.current.z=-1.5f;
            mavlink_log_critical(&mavlink_log_pub, "-- position -- 6");
            orb_publish(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_pub, &_pos_sp_triplet);
            timetick = hrt_absolute_time();
        }*/

        if(sended == 5 && hrt_absolute_time() - timetick > 10000000)//再过10秒关闭offboard模式切换到之前的模式
        //这就是为什么我们在起飞之前先设置为position control模式 这里切回之后就是position control mode
        {
            sended++;

            //退出offboard模式
            _command.command = vehicle_command_s::VEHICLE_CMD_NAV_GUIDED_ENABLE;
            _command.param1 = 0.0f;//0.0f表示关闭offboard 1.0f表示开启
			_command.param2 = 3.0f;//二级模式为position control
            _command.param3 = 0.0f;//三级模式没有！

            //修改原来的退出offboard 改为切换到land模式
            // _command.param1 = 213;//用参数213会出现一个问题：切换的时候会自动解锁
            // _command.param2 = 4;
            // _command.param3 = 6;
            // _command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;


            if (vehicle_command_pub != nullptr) {//发布命令
                orb_publish(ORB_ID(vehicle_command), vehicle_command_pub, &_command);
            } else {
                vehicle_command_pub = orb_advertise(ORB_ID(vehicle_command), &_command);
            }
            mavlink_log_critical(&mavlink_log_pub, "-- switch land mode");
            
        }

        //前面说过了 为了维持offboard模式必须要一个心跳信息最低美秒发布2次offboard_control_mode
        ocm.timestamp = hrt_absolute_time();//这里记得赋给最新的时间
        if (offboard_pub != nullptr) {
            orb_publish(ORB_ID(offboard_control_mode), offboard_pub, &ocm);
        } else {
            offboard_pub = orb_advertise(ORB_ID(offboard_control_mode), &ocm);
        }
        usleep(100000);//我们这里只睡眠100ms 所以理论上每秒发布了10次offboard_control_mode，offboard将不会自动关闭
    }

    mavlink_log_critical(&mavlink_log_pub, "[uart_net] -- exiting");

    thread_should_exit = false;
    thread_running = false;
    
    fflush(stdout);
    return 0;
}
