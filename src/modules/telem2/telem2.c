/* 
 * 2019年1月20日
 * 项目北京航天光华电子技术有限公司三栖无人机
 * 硬件普通的pixhawk的 源码v1.5.5
 * TELEM1 : /dev/ttyS1
 * TELEM2 : /dev/ttyS2
 * 使用telem2接受遥控器的数据
 * 1.需要屏蔽源码中 之前telem2 mavlink的使用
 * 2.编译此模块 启动此模块
 * 3.在此模块中接收遥控器数据 并解析处理
 */
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <string.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>    
#include <px4_config.h>  
#include <nuttx/sched.h>  
#include <px4_tasks.h>
#include <systemlib/param/param.h>

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <commander/px4_custom_mode.h>

#include <math.h> 


static bool thread_should_exit = false;     /**< daemon exit flag */  
static bool thread_running = false;     /**< daemon status flag */  
static int daemon_task;             /**< Handle of daemon task / thread */  
static int uart_init(char * uart_name);
static int set_uart_baudrate(const int fd, unsigned int baud);    


   
__EXPORT int telem2_main(int argc, char *argv[]);

/** 
 * daemon management mode. 
 */  
__EXPORT int telem2_app_main(int argc, char *argv[]);  
   
   
/** 
 * Print the correct usage. 
 */  
static void usage(const char *reason);  
   
static void  
usage(const char *reason)  
{  
    if (reason) {  
        warnx("%s\n", reason);  
    }  
   
    warnx("usage: daemon {start|stop|status} [-p <additional params>]\n\n");  
}  
   
/** 
 * The daemon app only briefly exists to start 
 * the background job. The stack size assigned in the 
 * Makefile does only apply to this management task. 
 * 
 * The actual stack size should be set in the call 
 * to task_create(). 
 */  
 
   






int set_uart_baudrate(const int fd, unsigned int baud)
{
    int speed;

    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
            warnx("ERR: baudrate: %d\n", baud);
            return -EINVAL;
    }

    struct termios uart_config;

    int termios_state;

    /* 以新的配置填充结构体 */
    /* 设置某个选项，那么就使用"|="运算，
     * 如果关闭某个选项就使用"&="和"~"运算
     * */
    tcgetattr(fd, &uart_config); // 获取终端参数

    /* clear ONLCR flag (which appends a CR for every LF) */
    uart_config.c_oflag &= ~ONLCR;// 将NL转换成CR(回车)-NL后输出。

    /* 无偶校验，一个停止位 */
    uart_config.c_cflag &= ~(CSTOPB | PARENB);// CSTOPB 使用两个停止位，PARENB 表示偶校验

     /* 设置波特率 */
    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetospeed)\n", termios_state);
        return false;
    }
    // 设置与终端相关的参数，TCSANOW 立即改变参数
    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
        warnx("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }

    return true;
}





int uart_init(char * uart_name)
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);
    /*Linux中，万物皆文件，打开串口设备和打开普通文件一样，使用的是open（）系统调用*/
    // 选项 O_NOCTTY 表示不能把本串口当成控制终端，否则用户的键盘输入信息将影响程序的执行
    if (serial_fd < 0) {
        err(1, "failed to open port: %s", uart_name);
        return false;
    }
    //printf("Open the %s\n",serial_fd);
    return serial_fd;
}




int telem2_app_main(int argc, char *argv[])
{
    
    //普通pixhawk 使用telem2接受数据
    int uart_read = uart_init("/dev/ttyS2");
    if(false == uart_read)
        return -1;
    if(false == set_uart_baudrate(uart_read,115200)){
        printf("set_uart_baudrate is failed\n");
        return -1;
    }
    

    thread_running = true;
    printf("telem2 starts successfully\n");


    int actuator_fd = orb_subscribe(ORB_ID(actuator_controls_0));
    orb_set_interval(actuator_fd, 100); //间隔时间单位ms

    struct manual_control_setpoint_s  manual; /**< controller status */
    memset(&manual, 0, sizeof(manual));
    orb_advert_t _manual_control_pub=NULL;

    struct vehicle_command_s vcmd;
    memset(&vcmd, 0, sizeof(vcmd));
    orb_advert_t _vehicle_command_pub=NULL;



    param_t sys_id_param = param_find("MAV_SYS_ID");
	param_t comp_id_param = param_find("MAV_COMP_ID");

	int32_t sys_id;
	int32_t comp_id;

	if (param_get(sys_id_param, &sys_id)) {
		errx(1, "PRM SYSID");
	}

	if (param_get(comp_id_param, &comp_id)) {
		errx(1, "PRM CMPID");
	}




    //遥控器数据的接收
    unsigned char temp=0x00; //用来临时保存接收的一个字节的遥控器数据，遥控器数据要for循环一个一个接收，不用担心有遗漏。
    unsigned char RC_rec[14]={0x00};//接收到的遥控器数据，主要保存正文部分
    unsigned char count=0;      //计数
    unsigned char check_data=0; //校验数据


    //发布飞行模式 这两个按键组合出四种飞行模式
    //                Rocker=0        Rocker=0=1
    //  button=偶数   自稳（mode=1）    定高（mode=2）  备注：使用gear_switch  区分三种不同的定高模式
    //  button=奇数   水下（mode=8）    水面（mode=4）
    unsigned char button=0;//检测按键按下 并松开
    bool Press=false;
    unsigned char Anti_shake=0;
    unsigned char mode=0;
    unsigned char mode_last=0;


    while (!thread_should_exit) { 

        // //向串口写数据，测试代码是OK的
        // char a[10]={'w','a','n','g','g','e','n','\n'};
        // int funk=0;
        // funk=write(uart_read,&a,8);
        // warnx("write byte = %d",funk);

        // //从串口读取数据，再把读取的数据写回串口，已经结合串口调试助手，回显的测试OK
        // char data=0x00;
        // read(uart_read,&data,1);
        // warnx("read data == %d\n",data);
      
        // write(uart_read,&data,1);
        // warnx("data has write back !");



        count = read(uart_read,&temp,1);

        if(temp==0xFF){ //开头是0xFF 0xFC

            count = read(uart_read,&temp,1);

            if(temp==0xFC){ //开头是0xFF 0xFC

                //接受遥控器的14个数据 第12个数据是校验 第13、14数据是FD、FD结束符
                for(int i=0;i<14;i++){
                    count = read(uart_read,&temp,1);
                    RC_rec[i]=temp;
                }

                // //把收集遥控器数据再写回去 对比下看看接收是否正确
                // count = write(uart_read,RC_rec,sizeof(RC_rec));
                   count=count;
                
                //对接收的数据进行校验
                check_data=0xFF^0xFC;
                for(int i=0;i<11;i++)
                {
                    check_data=check_data^RC_rec[i];
                }
                // warnx("rc11  %02x",RC_rec[11]);
                // warnx("check %02x",check_data);


                if(check_data==RC_rec[11]){
                   
                    //warnx("check ok");

                    for(int i=0;i<11;i++)
                    {
                        manual.channel[i]=RC_rec[i];//把获取的遥控器数据拷贝过来
                    }

                    //四个摇杆的范围是0-200,其中中间是100,这里归一化为-1到1
                    manual.x=(float)( RC_rec[0]/100.0 -1.0); 
                    manual.y=(float)( RC_rec[1]/100.0 -1.0);
                    manual.r=(float)( RC_rec[3]/100.0 -1.0);
                    //注意这里z轴范围归一化0-1，保持了和正常遥控器一样的范围，最低为0 最高为1 默认中间为0.5
                    //这种配置如果是飞STAB肯定有问题 但是如果飞定高就相当合适
                    //所以全局只飞在定高模式下，在定高模式区分：空中 水面 还是水下，至于真实的定高效果可能还需要优化
                    manual.z=(float)( RC_rec[2]/200.0 );   


                    //既然遥控器数据OK，下面就进行发布了                  
                    //用航天光华的遥控器数据进行替换，原来在sensor.cpp中进行发布，那里已经屏蔽


                    
 
                    //检测那个按键按下并松开 这一部分按键5  RC_rec[4]，按下检测 松开算数的检测已经没问题
                    if(RC_rec[4]==1){
                        Anti_shake++;
                        Press=true;
                    }
                    else if(Press&& (Anti_shake)>3){
                        Anti_shake=0;
                        Press=false;
                        button++;
                    }else{

                    }
                    //warnx("- %d",button);

                    //记录上一次的飞行模式
                    mode_last=mode;
                    //判断最新的飞行模式切换 这一部分模式切换没有问题
                    if ( button%2==0 ){//偶数
                        
                        if(RC_rec[5]==0){
                            mode=1; //mode=1定义为manual模式，下面会发布vehicle_commander切换到MANUAL
                                    //如果考虑光华遥控器和都是定高飞行，可以没有manual模式 可以在这里mode=2，就是只有ALT 在ALT下gear_swutch区分空中、水面、水下
                        }
                        else{
                            mode=2;
                        }
                        
                    }
                    else{
                        if(RC_rec[5]==1){
                            mode=4;
                        }
                        else{
                            mode=8;
                        }
                    }                  

                     manual.gear_switch=mode;
                    if (_manual_control_pub != NULL) {
                    	orb_publish(ORB_ID(manual_control_setpoint), _manual_control_pub, &manual);

                    } else {
                    	_manual_control_pub = orb_advertise(ORB_ID(manual_control_setpoint), &manual);
                    }

                    // //如果飞行模式发生了切换 发布
                    if(mode!=mode_last)
                    {   

                        //  自稳（mode=1）    定高（mode=2）  备注：使用gear_switch  区分三种不同的定高模式
                        //  水下（mode=8）    水面（mode=4）
                        //warnx("mode = %d",mode);

                        if(mode!=1){

                            vcmd.param1 = 29;//用参数213会出现一个问题：切换的时候会自动解锁
                            vcmd.param2 = PX4_CUSTOM_MAIN_MODE_ALTCTL;
                            vcmd.param3 = 0;
                            vcmd.command = 176;
                            vcmd.target_system = sys_id;
                            vcmd.target_component = comp_id;
                            vcmd.source_system = sys_id;
                            vcmd.source_component = comp_id;

                        }else{

                            vcmd.param1 = 29;//用参数213会出现一个问题：切换的时候会自动解锁
                            vcmd.param2 = PX4_CUSTOM_MAIN_MODE_MANUAL;
                            vcmd.param3 = 0;
                            vcmd.command = 176;
                            vcmd.target_system = sys_id;
                            vcmd.target_component = comp_id;
                            vcmd.source_system = sys_id;
                            vcmd.source_component = comp_id;

                        }


                        if (_vehicle_command_pub != NULL) {
                                orb_publish(ORB_ID(vehicle_command), _vehicle_command_pub, &vcmd);
                        } else {
                                _vehicle_command_pub = orb_advertise(ORB_ID(vehicle_command), &vcmd);
                        }
                    }




                }
                else{
                   warnx("check not ok");
                }


            }
        }




        







        //以下是根据控制量 往串口外写数据
        /*
        struct actuator_controls_s _actuator;
        memset(&_actuator, 0, sizeof(_actuator)); 
        bool updated2 = false; 
        orb_check(actuator_fd, &updated2);

        if (updated2)   
        {
            updated2 = false;       
            orb_copy(ORB_ID(actuator_controls_0), actuator_fd, &_actuator);
            warnx("已经接收到发布的控制量数据");

            yaw      = (int)(_actuator.control[2]*10000); //yaw放大10000倍相当于混控器的作用
            throttle = (int)(_actuator.control[3]*10000); //throttle放大10000倍相当于混控器的作用
            warnx("yaw      =%d ",yaw);
            warnx("throttle =%d \n",throttle);

            //转向的处理
            if( yaw<0 ){
                mode=0x20;   //左转
                yaw=-yaw;
                turn=yaw/40; //范围0-250
            }
            else if( yaw>0 ){
                mode=0x40;    //右转
                turn=yaw/40;  //范围0-250
            }
            else{
                mode=0; //不转
                turn=0;
            }

            warnx("yaw      =%d ",yaw);
            warnx("turn     =%d \n",turn);

            //油门的处理
            throttle= throttle/40 +250 ; //范围250-500
            thr_low=throttle*0x0F;        //油门低字节
            thr_high=throttle*0xF0;        //油门高字节

            warnx("throttle =%d ",throttle);
            warnx("thr_low  =%d ",thr_low);
            warnx("thr_high =%d \n\n\n",thr_high);
            
            output[3]=mode;
            output[4]=thr_high;
            output[5]=thr_low;
            output[6]=turn;

            // output[3]=0x20;
            // output[4]=0x01;
            // output[5]=0xF4;
            // output[6]=250;

            unsigned int temp=output[0]+output[1]+output[2]+output[3]+output[4]+output[5]+output[6]+output[7];

            output[8]=(unsigned char)(temp>>8);
            output[9]=(unsigned char)(temp&0x00FF);
            
            conut=write(uart_read,output,sizeof(output));
            warnx("write uart %d byte",conut);
        }    
        usleep(20000); 
        */

     }
                   
    
    printf("telem2 exits\n"); 
    thread_running = false;
    return 0;
}

int telem2_main(int argc, char *argv[])  
{  
    if (argc < 2) {  
        usage("missing command");  
        return 1;  
    }  
   
    if (!strcmp(argv[1], "start")) {  
   
        if (thread_running) {  
            warnx("daemon already running\n");  
            /* this is not an error */  
            return 0;  
        }  
   
        thread_should_exit = false;  
        daemon_task = px4_task_spawn_cmd("telem2",  
                         SCHED_DEFAULT,  
                         SCHED_PRIORITY_DEFAULT,  
                         2000,  
                         telem2_app_main,  
                         (argv) ? (char *const *)&argv[2] : (char *const *)NULL);  
        return 0;  
    }  
   
    if (!strcmp(argv[1], "stop")) {  
        thread_should_exit = true;
            thread_running = false;
        return 0;  
    }  
   
    if (!strcmp(argv[1], "status")) {  
        if (thread_running) {  
            warnx("\trunning\n");  
   
        } else {  
            warnx("\tnot started\n");  
        }  
   
        return 0;  
    }  
   
    usage("unrecognized command");  
    return 1;  
} 