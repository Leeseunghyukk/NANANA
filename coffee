#include <wiringPi.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <softTone.h>
#include <softPwm.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define MAXTIMINGS	85
#define DHTPIN		4
#define BUTTON          0
#define BUTTONS 5
#define BUTTONSS 6
#define LED 1
#define SERVO 2
#define BUZZER 3
#define SEG_A		21
#define SEG_B		22
#define SEG_C		23
#define SEG_D		24
#define SEG_E		25
#define SEG_F		27
#define SEG_G		28
#define SEG_DOT	29
#define SIZE    sizeof(struct sockaddr_in)
#define BUFSIZE 1024
int u=0;
int dht11_dat[5] = { 0, 0, 0, 0, 0 };
float	f; /* fahrenheit */
void *thread_function(void*arg);
void *threadBUT(void*arg);
void *threadBUZ(void*arg);
void *threadBUTS(void*arg);
void *threadBUTSS(void*arg);
void *threadDHT(void*arg);
void initPinMode();
int inputNumber (int hexVal);
int numPinID[8] = {SEG_A,SEG_B, SEG_C, SEG_D, SEG_E, SEG_F, SEG_G, SEG_DOT };
int numVal[10] = {0x3f,  0x06,  0x5b,  0x4f, 0x66, 0x6d, 0x7d, 0x27, 0x7f, 0x67};
void read_dht11_dat();

int main()
{

	if ( wiringPiSetup() == -1 )
		exit( 1 );

	pinMode(LED,OUTPUT);
	pinMode(BUTTON, INPUT);
	pinMode(BUTTONS, INPUT);
	pinMode(BUTTONSS, INPUT);
	initPinMode();
	pullUpDnControl(BUTTON, PUD_UP);
	pullUpDnControl(BUTTONS, PUD_UP);	
	pullUpDnControl(BUTTONSS, PUD_UP);	
	softPwmCreate(SERVO,0,200);
	int state, state1,state2, state3, state4,state5;
	pthread_t id1, id2, id3, id4, id5;
	void * t_return;
	state = pthread_create(&id1, NULL,threadBUT,NULL);	
	state2 = pthread_create(&id2, NULL, threadBUZ,NULL); 
	state3 = pthread_create(&id3,NULL,threadBUTS,NULL);	
	state4 = pthread_create(&id4,NULL,threadDHT,NULL);
	state5 = pthread_create(&id5,NULL,threadBUTSS,NULL);
	int sockfd_connect, sockfd_listen;
	char recv1[BUFSIZE];
	int tem;


	struct sockaddr_in server = {AF_INET, 33333, INADDR_ANY};
	if((sockfd_listen = socket(AF_INET, SOCK_STREAM, 0))==-1) {
		puts("fail to call socket()"); exit(1);
	}
	if(bind(sockfd_listen, (struct sockaddr *)&server, SIZE)==-1) {
		puts("fail to call bind()"); exit(1);
	}
	if(listen(sockfd_listen, 5)==-1) {
		puts("fail to call listen()"); exit(1);
	}


	while(1) {

		if((sockfd_connect = accept(sockfd_listen, NULL, NULL))==-1) {
			puts("fail to call accept()"); continue;
		} 

		while(read(sockfd_connect, recv1, BUFSIZE)>0) {

			tem = f;

			send(sockfd_connect, &tem, BUFSIZE, 0);

		} if(read(sockfd_connect, recv1, BUFSIZE)<0)  {
			puts("client is not operating");
			break;                
		}

	}
	if (state != 0)
	{
		puts("error");
		exit(1);
	}

	return(0);
}
void *threadDHT(void *arg)
{
	while(1)
	{	read_dht11_dat();
	delay( 2000 ); /* wait 1sec to refresh */
	}		

}
void *threadBUT(void *arg)
{
	int p;
	while(1)
	{
		p=digitalRead(BUTTON);
		if(p ==LOW)
		{
			u=u+1;	
			inputNumber(numVal[u]);
			delay(500);	
			if( u > 5)
			{
				u=1;
				inputNumber(numVal[u]);
			}
		}
		else
		{
		}
	}
}


void *threadBUZ(void*arg)
{
	int z;
	int scale[1] = {980};

	softToneCreate(BUZZER);
	while(1)
	{
		if(f >= 53)
		{

			z = 1;
			softToneWrite(BUZZER, scale[z]);
			delay(50);
		}
		else
		{
		}


	}
}
void *threadBUTS(void*arg)
{
	int o;
	int m;

	while(1)
	{
		o=digitalRead(BUTTONS);

		if( o == LOW)
		{	digitalWrite(LED, HIGH);	
		u=u;

		char str;


		for(m=0; m <100; m++)
		{	

			softPwmWrite(SERVO,5); 
			delay(2000/u);
			softPwmWrite(SERVO,100);
			delay(2000/u);

		}	
		}
		else 
		{	
			digitalWrite(LED,LOW);

		}		
	}

}
void *threadBUTSS(void*arg)
{	
	while(1){
		int n=digitalRead(BUTTONSS);
		if( n == LOW)
		{
			exit(1);
		}
		else
		{
		}
	}
}

void initPinMode() {
	for(int y=0; y<8; y++) {
		pinMode(numPinID[y], OUTPUT);
	}

}

int inputNumber(int hexVal) {
	int y=0;
	unsigned char buf[8];
	memset(buf, 0, 8);
	if( (hexVal > 0xff) | (hexVal < 0x00) ) {
		printf("ERR : Value Overflow\n");
		return -1;
	}
	while(1) {
		buf[y] = hexVal%2;
		hexVal = hexVal/2;
		y++;

		if(hexVal <= 0) {
			break;
		}
	}
	for(y=0; y<8; y++) {
		digitalWrite(numPinID[y], buf[y]);
	}		

}

void read_dht11_dat()
{
	uint8_t laststate	= HIGH;
	uint8_t counter		= 0;
	uint8_t j		= 0, i;


	dht11_dat[0] = dht11_dat[1] = dht11_dat[2] = dht11_dat[3] = dht11_dat[4] = 0;

	/* pull pin down for 18 milliseconds */
	pinMode( DHTPIN, OUTPUT );
	digitalWrite( DHTPIN, LOW );
	delay( 18 );
	/* then pull it up for 40 microseconds */
	digitalWrite( DHTPIN, HIGH );
	delayMicroseconds( 40 );
	/* prepare to read the pin */
	pinMode( DHTPIN, INPUT );

	/* detect change and read data */
	for ( i = 0; i < MAXTIMINGS; i++ )
	{
		counter = 0;
		while ( digitalRead( DHTPIN ) == laststate )
		{
			counter++;
			delayMicroseconds( 1 );
			if ( counter == 255 )
			{
				break;
			}
		}
		laststate = digitalRead( DHTPIN );

		if ( counter == 255 )
			break;

		/* ignore first 3 transitions */
		if ( (i >= 4) && (i % 2 == 0) )
		{
			/* shove each bit into the storage bytes */
			dht11_dat[j / 8] <<= 1;
			if ( counter > 50 )
				dht11_dat[j / 8] |= 1;
			j++;
		}
	}

	/*
	* check we read 40 bits (8bit x 5 ) + verify checksum in the last byte
	* print it out if data is good
	*/
	if ( (j >= 40) &&
		(dht11_dat[4] == ( (dht11_dat[0] + dht11_dat[1] + dht11_dat[2] + dht11_dat[3]) & 0xFF) ) )
	{
		f = dht11_dat[2] * 9. / 5. + 32;
		printf( "Humidity = %d.%d %% Temperature = %d.%d *C (%.1f *F)\n",
			dht11_dat[0], dht11_dat[1], dht11_dat[2], dht11_dat[3], f );
	}else  {
		printf( "Sensing a Temperature\n" );
	}



}
