/* 
 * 2019年1月20日
 * 项目北京航天光华电子技术有限公司三栖无人机
 * 硬件普通的pixhawk的 源码v1.5.5
 * TELEM1 : /dev/ttyS1
 * ms5837 : /dev/ttyS2
 * 使用ms5837接受遥控器的数据
 * 1.需要屏蔽源码中 之前ms5837 mavlink的使用
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

#include <uORB/topics/manual_control_setpoint.h>


#include <math.h> 


static bool thread_should_exit = false;     /**< daemon exit flag */  
static bool thread_running = false;     /**< daemon status flag */  
static int daemon_task;             /**< Handle of daemon task / thread */  
static int uart_init(char * uart_name);
static int set_uart_baudrate(const int fd, unsigned int baud);    


   
__EXPORT int ms5837_main(int argc, char *argv[]);

/** 
 * daemon management mode. 
 */  
__EXPORT int ms5837_app_main(int argc, char *argv[]);  
   
   
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




int ms5837_app_main(int argc, char *argv[])
{
     
    //普通pixhawk 使用serila4/5中的串口4 接收ms5837解算板的深度数据
    //serila4/5中的串口4对应的是ttyS6
    int uart_read = uart_init("/dev/ttyS6");
    if(false == uart_read)
        return -1;
    if(false == set_uart_baudrate(uart_read,115200)){
        printf("set_uart_baudrate is failed\n");
        return -1;
    }
   
    

    thread_running = true;
    printf("ms5837 starts successfully\n");



    // struct manual_control_setpoint_s  manual; /**< controller status */
    // memset(&manual, 0, sizeof(manual));
    // orb_advert_t _manual_control_pub=NULL;
    
    float temper=0.0f;
    float deep=0.0f;


    while (!thread_should_exit) { 

 
        // //向串口写数据，测试代码是OK的
        // char a[10]={'w','a','n','g','g','e','n','\n'};
        // int funk=0;
        // funk=write(uart_read,&a,8);
        // warnx("write byte = %d",funk);


        //串口是阻塞等数据的，当没有数据的时候 代码就会停止在read这个函数
        //串口是阻塞等数据的，当没有数据的时候 代码就会停止在read这个函数
        //串口是阻塞等数据的，当没有数据的时候 代码就会停止在read这个函数

        char data=0x00;
        char temp[8]={0x00};
        char depth[8]={0x00};
        read(uart_read,&data,1);
        if(data=='T'){
            read(uart_read,&data,1);
            if(data=='='){
                //下面是温度数据的获取
                for(int i=0;i<5;i++){
                    read(uart_read,&data,1);
                    temp[i]=data;
                }
                temper = atof(temp);
                //warnx("%2.4f",(double)temper);

            }

            read(uart_read,&data,1);
            if(data=='D'){
                read(uart_read,&data,1); 
                if(data=='='){
                    for(int i=0;i<6;i++){
                        read(uart_read,&data,1);
                        depth[i]=data;
                    }
                }
                deep=atof(depth);
               // warnx("%3.4f",(double)deep);
            }


        }
       
     }
                   
    
    printf("ms5837 exits\n"); 
    thread_running = false;
    return 0;
}

int ms5837_main(int argc, char *argv[])  
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
        warnx("1"); 
   
        thread_should_exit = false;  
        daemon_task = px4_task_spawn_cmd("ms5837",  
                         SCHED_DEFAULT,  
                         SCHED_PRIORITY_DEFAULT,  
                         1200,  
                         ms5837_app_main,  
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