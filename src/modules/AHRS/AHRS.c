/* 
 * 串口读取函数
 * rw_uart.c 
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
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/ekf2_innovations.h>
#include <uORB/topics/mount_orientation.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <math.h> 


static bool thread_should_exit = false;     /**< daemon exit flag */  
static bool thread_running = false;     /**< daemon status flag */  
static int daemon_task;             /**< Handle of daemon task / thread */  
static int uart_init(char * uart_name);
static int set_uart_baudrate(const int fd, unsigned int baud);    
void quat_to_dcm(void);
void dcm_to_euler(void);
//struct vehicle_global_position_s position;
//struct vehicle_attitude_setpoint_s attitude;
float dcm[3][3];
float q[4];
float q00,q11,q22,q33,q01,q02,q03,q12,q13,q23;
float euler[3];
   
__EXPORT int AHRS_main(int argc, char *argv[]);

/** 
 * daemon management function. 
 */  
__EXPORT int AHRS_app_main(int argc, char *argv[]);  
   
   
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
//    printf("Open the %s\n",serial_fd);
    return serial_fd;
}

void quat_to_dcm(void)
{
    q00 = q[0]*q[0];
    q11 = q[1]*q[1];
    q22 = q[2]*q[2];
    q33 = q[3]*q[3];
    q01 = q[0]*q[1];
    q02 = q[0]*q[2];
    q03 = q[0]*q[3];   
    q12 = q[1]*q[2];
    q13 = q[1]*q[3];
    q23 = q[2]*q[3];   

    dcm[0][0] = q00 + q11 - q22 - q33;
    dcm[1][1] = q00 - q11 + q22 - q33;
    dcm[2][2] = q00 - q11 - q22 + q33;
    dcm[0][1] = 2.0f * (q12 - q03);
    dcm[0][2] = 2.0f * (q13 + q02);
    dcm[1][0] = 2.0f * (q12 + q03);
    dcm[1][2] = 2.0f * (q23 - q01);
    dcm[2][0] = 2.0f * (q13 - q02);
    dcm[2][1] = 2.0f * (q23 + q01);
}

void dcm_to_euler(void)
	{
		euler[1] = asinf(-dcm[2][0]);

		if (fabsf(euler[1] - M_PI_2_F) < 1.0e-3f) {
			euler[0] = 0.0f;
			euler[2] = atan2f(dcm[1][2] - dcm[0][1], dcm[0][2] + dcm[1][1]) + euler[0];

		} else if (fabsf(euler[1] + M_PI_2_F) < 1.0e-3f) {
			euler[0] = 0.0f;
			euler[2] = atan2f(dcm[1][2] - dcm[0][1], dcm[0][2] + dcm[1][1]) - euler[0];

		} else {
			euler[0] = atan2f(dcm[2][1], dcm[2][2]);
			euler[2] = atan2f(dcm[1][0], dcm[0][0]);
		}

	}

int AHRS_app_main(int argc, char *argv[])
{
    /*
     * TELEM1 : /dev/ttyS1
     * TELEM2 : /dev/ttyS2
     * GPS2    :/dev/ttyS6
     *         : /dev/ttyS5
     *         : /dev/ttyS6
     * N/A    : /dev/ttyS4
     * IO DEBUG (RX only):/dev/ttyS0
     */
    /////////////////////////////////////////////////////////标注的是串口的输出程序 使用的赫星pixhakw2板子 代码px4 1.7.0 串口gps2对应ttys6
    int uart_read = uart_init("/dev/ttyS6");
    if(false == uart_read)
        return -1;
    if(false == set_uart_baudrate(uart_read,115200)){
        printf("set_uart_baudrate is failed\n");
        return -1;
    }
    //////////////////////////////////////////////////////////
    thread_running = true;
    printf("AHRS starts successfully\n");
 //   int position_fd = orb_subscribe(ORB_ID(vehicle_global_position));
    //int attitude_fd = orb_subscribe(ORB_ID(ekf2_innovations));

  //  orb_copy(ORB_ID(vehicle_global_position), position_fd, &position);
    int attitude_fd = orb_subscribe(ORB_ID(vehicle_attitude));
    int position_fd = orb_subscribe(ORB_ID(vehicle_global_position));
    int sensor_fd = orb_subscribe(ORB_ID(sensor_combined));
    int gps_fd = orb_subscribe(ORB_ID(vehicle_gps_position));
  //  orb_set_interval(attitude_fd, 500);
    orb_set_interval(gps_fd, 500);        
    while (!thread_should_exit) { 


////////////////////////////////////////////标注的是串口的输出程序 使用的赫星pixhakw2板子 代码px4 1.7.0 串口gps2对应ttys6
        // char a[10]={'w','a','n','g','g','e','n','\n'};
        // int funk=0;
        // funk=write(uart_read,&a,8);
        // warnx("--%d",funk);
///////////////////////////////////////////


        bool attitude_updated = false;  
        bool position_updated = false; 
        bool sensor_updated = false;
        bool gps_updated = false;        
        orb_check(attitude_fd, &attitude_updated);
        orb_check(position_fd, &position_updated);       
        orb_check(sensor_fd, &attitude_updated);
        orb_check(gps_fd, &gps_updated);  
     /* obtained data for the first file descriptor */  
        attitude_updated = false;
        struct vehicle_attitude_s attitude; 
        struct vehicle_global_position_s position;     
        struct sensor_combined_s sensor;  
        struct vehicle_gps_position_s gps;     
        memset(&attitude, 0, sizeof(attitude)); 
        memset(&position, 0, sizeof(position)); 
        memset(&sensor, 0, sizeof(sensor)); 
        memset(&gps, 0, sizeof(gps));
      /* copy sensors raw data into local buffer */  
        if (attitude_updated)
        {
            attitude_updated = false;
            orb_copy(ORB_ID(vehicle_attitude), attitude_fd, &attitude);
            q[0] = attitude.q[0];
            q[1] = attitude.q[1];
            q[2] = attitude.q[2];
            q[3] = attitude.q[3];
            quat_to_dcm();
            dcm_to_euler();
        }
        if (position_updated)
        {
            position_updated = false;   
            orb_copy(ORB_ID(vehicle_global_position), position_fd, &position);         
        }
        if (sensor_updated)
        {
            sensor_updated = false;   
            orb_copy(ORB_ID(sensor_combined), sensor_fd, &sensor);                  
        }
        if (gps_updated)   
        {
            gps_updated = false;       
            orb_copy(ORB_ID(vehicle_gps_position), gps_fd, &gps);
                    short t = 0x55AA;
        write(uart_read,&t,2);
      //  write(uart_read,&attitude.baro_alt_meter,4);
        write(uart_read,&gps.fix_type,1);
        }    
        usleep(20000); 

     }
                   
    
    printf("AHRS exits\n"); 
    thread_running = false;
    return 0;
}

int AHRS_main(int argc, char *argv[])  
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
        daemon_task = px4_task_spawn_cmd("AHRS",  
                         SCHED_DEFAULT,  
                         SCHED_PRIORITY_DEFAULT,  
                         2000,  
                         AHRS_app_main,  
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