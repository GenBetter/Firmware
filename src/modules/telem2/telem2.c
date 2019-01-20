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

#include <uORB/topics/actuator_controls.h>

#include <math.h> 


static bool thread_should_exit = false;     /**< daemon exit flag */  
static bool thread_running = false;     /**< daemon status flag */  
static int daemon_task;             /**< Handle of daemon task / thread */  
static int uart_init(char * uart_name);
static int set_uart_baudrate(const int fd, unsigned int baud);    


int conut=0;
int  yaw=0x00;     //存储yaw控制量放大10000倍 相当于混控器的作用
int  throttle=0x00;//存储throttle控制量放大10000倍 相当于混控器的作用

unsigned char mode=0x00;//功能字节mode 左转 右转
unsigned char turn=0x00;//转弯的pwm 0-250

unsigned char thr_low=0x00; //油门低字节
unsigned char thr_high=0x00; //油门高字节

unsigned char output[10]={0x00};//最终的串口输出

   
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
    if(false == set_uart_baudrate(uart_read,57600)){
        printf("set_uart_baudrate is failed\n");
        return -1;
    }
    

    thread_running = true;
    printf("telem2 starts successfully\n");


    int actuator_fd = orb_subscribe(ORB_ID(actuator_controls_0));
    orb_set_interval(actuator_fd, 100); //间隔时间单位ms


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