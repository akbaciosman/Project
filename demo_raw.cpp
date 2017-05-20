#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <iostream>
#include  <fcntl.h>

//#include <sys/socket.h>
#include <stdio.h>
#include <netinet/in.h> 
#include <string.h> //for memset
#include <unistd.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <pthread.h>
#include <errno.h>
#include <signal.h>
//for struct sockaddr_in and socket parameters



#define PI 3.14159265
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;


using namespace std;

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

void setup();
double dist(double a,double b);
double get_x_rotation(double x,double y,double z);
double get_y_rotation(double x,double y,double z);
void loop();
int setupSocket();


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
    printf("xx %lf yy %lf\n",x_diff,y_diff);
    if(x_diff>0.9 || y_diff>0.9){   
    	printf("sent byte:%d\n",(int)write(sockfd,&mess,sizeof(strlen(mess))));
    	printf("sent byte:%d\n",(int)write(sockfd,&buffer,sizeof(strlen(buffer))));
    	printf("g:%lf %lf \n",x_rot,y_rot);
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


int main()
{
    setup();
    if(setupSocket()!=1)
	return -1;
    for (;;)
        loop();

	
   return 1;
}

void signalHandler(int sig)
{	//char  send_message[1024] = "exit";
	//write(connfd,&send_message,1024);
			/*-----close connection-----*/
	char mess[1024] = "exit";
	//write(sockfd,&mess,sizeof(strlen(mess)))
	//close(connfd);
	//close(sockfd);
		
	printf("Terminated connection\n");
	printf("-----------------END------------------\n\n");

    exit(0);
}
