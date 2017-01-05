#include "Protocentral_MAX30102.h"
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

#define NUMPIXELS      60
#define BRIGHTNESS_DIVISOR 8  //to lower the max brightness of the neopixel LED
#define MAX_BRIGHTNESS 255

Adafruit_NeoPixel LED = Adafruit_NeoPixel(NUMPIXELS, 5, NEO_GRB + NEO_KHZ800);

#if defined(ARDUINO_AVR_UNO)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated.  Samples become 16-bit data.
uint16_t aun_ir_buffer[100]; //infrared LED sensor data
uint16_t aun_red_buffer[100];  //red LED sensor data
#else
uint32_t aun_ir_buffer[100]; //infrared LED sensor data
uint32_t aun_red_buffer[100];  //red LED sensor data
#endif
int32_t n_ir_buffer_length; //data length


MAX30100 sensor;  //attach sensor
uint8_t data_len=8;      
uint8_t DataPacketHeader[15];
volatile unsigned int IRR,REDD;
volatile bool blinkflag =0;
volatile long blinktime=0;
volatile long t=0;
volatile float current_time=0 ;
uint32_t un_min, un_max, un_prev_data, un_brightness;  //variables to calculate the on-board LED brightness that reflects the heartbeats
int32_t i;
float f_temp;


void setup()
{
  Wire.begin();
  Serial.begin(57600);
  sensor.begin(pw1600, i50, sr100 );
  pinMode(13, INPUT);         //pin D10 connects to the interrupt output pin of the MAX30102
  LED.begin();
  LED.show();
  LED.setBrightness(200);
}

void loop() 
{
    sensor.readSensor();        //read sensor
    IRR=sensor.IR;              
    REDD=sensor.RED;    

    un_brightness=0;
    un_min=0x3FFFF;
    un_max=0;
  
    n_ir_buffer_length=100;  //buffer length of 100 stores 4 seconds of samples running at 25sps

   //read the first 100 samples, and determine the signal range
    for(i=0;i<n_ir_buffer_length;i++)
    {
      while(digitalRead(13)==1);  //wait until the interrupt pin asserts
      
      if(un_min>aun_red_buffer[i])
        un_min=aun_red_buffer[i];  //update signal min
      if(un_max<aun_red_buffer[i])
        un_max=aun_red_buffer[i];  //update signal max
      Serial.print(F("red="));
      Serial.print(aun_red_buffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.println(aun_ir_buffer[i], DEC);
    }
    un_prev_data=aun_red_buffer[i];
    //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
 
  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
   while(1)
   {
      i=0;
      un_min=0x3FFFF;
      un_max=0;
  
      //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
      
      for(i=25;i<100;i++)
      {
        aun_red_buffer[i-25]=aun_red_buffer[i];
        aun_ir_buffer[i-25]=aun_ir_buffer[i];
  
        //update the signal min and max
        if(un_min>aun_red_buffer[i])
          un_min=aun_red_buffer[i];
        if(un_max<aun_red_buffer[i])
          un_max=aun_red_buffer[i];
      }
  
      //take 25 sets of samples before calculating the heart rate.
      for(i=75;i<100;i++)
      {
        un_prev_data=aun_red_buffer[i-1];
        sensor.readSensor();        //read sensor
        REDD=sensor.RED;    
        aun_red_buffer[i] = REDD;          
  
        //calculate the brightness of the LED
        if(aun_red_buffer[i]>un_prev_data)
        {
          f_temp=aun_red_buffer[i]-un_prev_data;
          f_temp/=(un_max-un_min);
          f_temp*=MAX_BRIGHTNESS;
          f_temp=un_brightness-f_temp;
          if(f_temp<0)
            un_brightness=0;
          else
            un_brightness=(int)f_temp;
        }
        else
        {
          f_temp=un_prev_data-aun_red_buffer[i];
          f_temp/=(un_max-un_min);
          f_temp*=MAX_BRIGHTNESS;
          un_brightness+=(int)f_temp;
          if(un_brightness>MAX_BRIGHTNESS)
           un_brightness=MAX_BRIGHTNESS;
        }

  for(int i=0;i<NUMPIXELS;i++){
      LED.setPixelColor(i, 0, un_brightness/BRIGHTNESS_DIVISOR, 0);
      LED.show();

  }
      
    DataPacketHeader[0] = 0x0A;
    DataPacketHeader[1] = 0xFA;
    DataPacketHeader[2] = (uint8_t) (data_len);
    DataPacketHeader[3] = (uint8_t) (data_len>>8);
    DataPacketHeader[4] = 0x02;
    
 
    DataPacketHeader[5] = REDD;
    DataPacketHeader[6] = REDD>>8;
    DataPacketHeader[7] = REDD>>16;
    DataPacketHeader[8] = REDD>>24; 

    
    DataPacketHeader[9] = IRR;
    DataPacketHeader[10] = IRR>>8;
    DataPacketHeader[11] = IRR>>16;
    DataPacketHeader[12] = IRR>>24; 

    DataPacketHeader[13] = 0x00;
    DataPacketHeader[14] = 0x0b;

    for(int i=0; i<15; i++) // transmit the data
    {
       Serial.write(DataPacketHeader[i]);
     }
   delay(40);
  }

  }
}
