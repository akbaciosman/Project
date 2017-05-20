#include <iostream>
#include <stdint.h>
#include <pigpio.h>
#include <unistd.h>
#include <string>
#include <string.h>

#include <stdio.h>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include  <fcntl.h>


#include <sys/socket.h>
#include <stdio.h>
#include <netinet/in.h> 
#include <arpa/inet.h>
#include <stdlib.h>
#include <pthread.h>
#include <errno.h>
#include <signal.h>


#define PI 3.14159265
MPU6050 accelgyro;

//Socket
#ifndef DEBUG
#define DEBUG 0
#endif
int connfd;
int sockfd = 0;
void signalHandler(int sig);

double x_rot_x=0.0;
double y_rot_y=0.0;
//RT_TASK demo_task;
int16_t ax, ay, az;
int16_t gx, gy, gz;

pthread_t *p1; //thread declaration

using namespace std;
#define increment "w"
#define decrement "s"
#define stop "9"

//mpu6050 socket declarations
void setup();
double dist(double a,double b);
double get_x_rotation(double x,double y,double z);
double get_y_rotation(double x,double y,double z);
void loop();
int setupSocket();

//motor declarations
void initiliase();
int update(int m1,int m2,int m3,int m4);
void *myfunc(void *arg);


//mpu6050 functions definitions
void setup() {
    // initialize device
    printf("Initializing I2C devices...\n");
    accelgyro.initialize();

    // verify connection
    printf("Testing device connections...\n");

    printf(accelgyro.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");
}

// read raw accel/gyro measurements from device
void loop() {

    //unutma
    //accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    accelgyro.getAcceleration(&gx, &gy, &gz);
    //accelgyro.getRotation(&gx, &gy, &gz);
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    char buffer [1024]="";//x_data
    double x_diff=0.0;
    double y_diff=0.0;
    x = gx / 16384.0;
    y = gy / 16384.0;
    z = gz / 16384.0;

    // display accel/gyro x/y/z values
    double x_rot = get_x_rotation(x,y,z);
    double y_rot = get_y_rotation(x,y,z);
    int size_x=sizeof(x_rot);
    int size_y=sizeof(y_rot);
    char mess[1024]="";//y_data
    x_diff = fabs(x_rot_x-x_rot);
    y_diff = fabs(y_rot_y-y_rot);
    sprintf(buffer,"%.5lf x",x_rot);
    sprintf(mess,"%.5lf y",y_rot);
    //printf("xx %lf yy %lf\n",x_diff,y_diff);
    if(x_diff>0.9 || y_diff>0.9){
	write(sockfd,&mess,sizeof(strlen(mess)));
	write(sockfd,&buffer,sizeof(strlen(buffer)));
        //printf("sent byte:%d\n",(int)write(sockfd,&mess,sizeof(strlen(mess))));
        //printf("sent byte:%d\n",(int)write(sockfd,&buffer,sizeof(strlen(buffer))));
        //printf("g:%lf %lf \n",x_rot,y_rot);
    }

    x_rot_x = x_rot;
    y_rot_y = y_rot;

}

double dist(double a,double b){

   return sqrt(a*a + b*b);

}

double get_x_rotation(double x,double y,double z){
        double radians = atan2(y,dist(x,z));
        return (double)(radians * (180/PI));
}

double get_y_rotation(double x,double y,double z){
        double radians = atan2(x,dist(y,z));
        return (double)((radians * (180/PI))*-1);
}


int setupSocket(){
        char recvBuff[1024],sendBuff[1026];
        struct sockaddr_in serv_addr;
        char  mess[1024] ;
        int len=0;
        char *temp;
        //strcpy(mess,"172.16.5.187,Osman\t");

        /*------initiliaze timeout for socket ---------*/
        struct timeval timeout;      
        timeout.tv_sec = 0;
        timeout.tv_usec = 3;
        /*--------------------*/

        signal(SIGINT, signalHandler);

        /*------create socket --------*/
        if((sockfd = socket(AF_INET, SOCK_STREAM, 0))< 0){
                perror("Error create socket  ");
                return -1;
        }else if(DEBUG){
                printf("Socket create succes (receive message)\n");
        }

        /*------assign socket for timeout------*/
        setsockopt(sockfd,SOL_SOCKET,SO_REUSEADDR,
               (char *)&timeout, sizeof(timeout));

        /*------------server address  set ------*/
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(8080);
        serv_addr.sin_addr.s_addr = inet_addr("10.1.18.72");



        printf("-----------------Send message ------------------\n");

        //if come a request to socket ,accept
		if((connfd=connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)))<0){
            perror("Error connecting socket ");
            return -1;
        }else{
            printf("Accept socket\n");
            return 1;
        }
}


void signalHandler(int sig)
{       //char  send_message[1024] = "exit";
        //write(connfd,&send_message,1024);
                        /*-----close connection-----*/
        char mess[1024] = "exit";
        //write(sockfd,&mess,sizeof(strlen(mess)))
        //close(connfd);
        //close(sockfd);
	gpioStopThread(p1);
	gpioTerminate();
        printf("Terminated connection\n");
        printf("-----------------END------------------\n\n");

    exit(0);
}


//motor
void initiliase(){
	gpioSetMode(18, PI_OUTPUT);
        gpioSetMode(23, PI_OUTPUT);
        gpioSetMode(24, PI_OUTPUT);
        gpioSetMode(25, PI_OUTPUT);


        gpioServo(18, 1000); // Now 2000 is fully on
        gpioServo(23, 1000); // Now 2000 is fully on
        gpioServo(24, 1000); // Now 2000 is fully on
        gpioServo(25, 1000); // Now 2000 is fully on

        sleep(1);


}


int update(int m1,int m2,int m3,int m4){
        gpioServo(18, m1); // Now 2000 is fully on
        gpioServo(23, m2); // Now 2000 is fully on
        gpioServo(24, m3); // Now 2000 is fully on
        gpioServo(25, m4); // Now 2000 is fully on
}

void *myfunc(void *arg){
	string command;
        int pulse=1300;

	initiliase();

	while(1){
                if(command.compare(increment)==0){
                        cout<<"increment"<<endl;
                        pulse+=100;
                }
                else if(command.compare(decrement)==0){
                        cout<<"decrement"<<endl;
                        pulse -= 100;
                }
                else if(command.compare(stop)==0){
                        cout<<"stop motors"<<endl;
                        pulse=1000;
                }
                cout<<"pulse : "<<pulse<<endl;
                command="";
                update(pulse,pulse,pulse,pulse);
                cout<<"Please enter command>>";
                cin>>command;
                if(pulse==1800)
                        break;
                //sleep(2);
        }

}

int main(int argc, char *argv[]){
	pthread_t *p1;

	if (gpioInitialise() < 0) return 1;
	setup();
        if(setupSocket()!=1)
                return -1;

	p1 = gpioStartThread(myfunc,(void *) "thread 1"); sleep(3);

   	update(1300,1300,1300,1300);
    	for (;;)
        	loop();

	return 0;
}
