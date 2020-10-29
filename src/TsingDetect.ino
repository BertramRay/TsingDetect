/*************************************************** 
TsingDetect Beta 1.0
Designer: Bertram Ray
Design Date: 2020/10/25
All Rights Reserved

Operation Description
1. Press Button 1 on app:
    Switch to the heart rate mode. After red LED
    lights, put your index finger on the sensor.
    About 7 seconds later, buzzer beeps, your phone
    will receive the BPM data and SPO2 data.
2. Press Button 2 on app:
    Switch to the temperature mode. After blue LED
    lights, put you index finger on the sensor.
    About 4 seconds later, buzzer beeps, your phone
    will receive the calibrated finger temperature 
    data.
3. Press Button 3 on app:
    Switch to the pulse wave mode. After blue LED
    lights, put you index finger on the sensor.
    About 4 seconds later, your PC serial ploter will receive
    the IR data and we will show this data using 
    immediate line diagram.
 ****************************************************/

#include <Wire.h>
#include <SoftwareSerial.h>
#include "Adafruit_MLX90614.h"
#include "MAX30105.h"
#include "heartRate.h"

//pin define
#define LED_blue 3
#define LED_red 4
#define LED_green 5
#define buzzer 6
#define temperature_power 7
#define heart_rate_power 8
#define bluetooth_transmit 12
#define bluetooth_receive 13

//operable object
uint8_t MLX90614_address = 0;
uint8_t MAX30105_address = 87;
MAX30105 particleSensor;
SoftwareSerial softSerial1(bluetooth_transmit,bluetooth_receive);
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

//heart_rate_data
const byte RATE_SIZE = 10; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg = 0;
int lastBeatAvg = 0;
int i = 0;
int count = 0;
boolean rate_validation_flag = 0;

//temperature data
double ta=0;
double tb=0;
const int t_size = 30;
double last_tb = 0;
boolean temperature_validation_flag = 0;


int mode=1;
boolean LED_mode=0;
boolean beep_mode=0;
int message;

void beep(){
    //buzzer beep
    if(beep_mode){
      digitalWrite(buzzer, LOW);
      delay(100);
      digitalWrite(buzzer, HIGH);
    }
}
void(* resetFunc) (void) = 0;

void setup() {
  Serial.begin(9600);
  softSerial1.begin(9600);
  softSerial1.println("初始化...");
  pinMode(heart_rate_power,OUTPUT);
  pinMode(temperature_power,OUTPUT);
  pinMode(buzzer,OUTPUT);
  pinMode(LED_red,OUTPUT);
  pinMode(LED_blue,OUTPUT);
  pinMode(LED_green,OUTPUT);
  digitalWrite(heart_rate_power, HIGH);
  digitalWrite(temperature_power, LOW);
  digitalWrite(buzzer, HIGH);
  digitalWrite(LED_red, LOW);
  digitalWrite(LED_blue, LOW);
  digitalWrite(LED_green, LOW);

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST, MAX30105_address)) //Use default I2C port, 400kHz speed
  {
    softSerial1.println("未检测到MAX30105模块 ");
    while (1);
  }

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  softSerial1.println("初始化完成。");
  softSerial1.println("欢迎使用[清测康]居家心率体温检测设备，默认为[心率模式]。");
  softSerial1.println("请将手指放入松紧带。");

}

void loop() {
  if(mode==1){
    long irValue = particleSensor.getIR();
    count++;
    if(count==10){
      digitalWrite(LED_red, LOW);
      digitalWrite(LED_blue, LOW);
      digitalWrite(LED_green, LOW);
    }
    if(count==3){
      digitalWrite(buzzer, HIGH);
    }
    if (checkForBeat(irValue) == true)
    {
      //We sensed a beat!
      long delta = millis() - lastBeat;
      lastBeat = millis();
  
      beatsPerMinute = 60 / (delta / 1000.0);
  
      if (beatsPerMinute < 255 && beatsPerMinute > 20)
      {
        if(LED_mode){
          if(beatsPerMinute > 20&&beatsPerMinute <= 45){
            digitalWrite(LED_red, HIGH);
          }else if(beatsPerMinute > 45&&beatsPerMinute <= 100){
            digitalWrite(LED_green, HIGH);
          }else if(beatsPerMinute > 100&&beatsPerMinute <= 255){
            digitalWrite(LED_blue, HIGH);
          }
        }
        if(beep_mode){
          digitalWrite(buzzer, LOW);
        }
        count=0;
        rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
        rateSpot %= RATE_SIZE; //Wrap variable
  
        //Take average of readings
        lastBeatAvg = beatAvg;
        beatAvg = 0;
        for (byte x = 0 ; x < RATE_SIZE ; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
        if(rate_validation_flag == 0){
          if((beatsPerMinute-beatAvg)<3&&(beatsPerMinute-beatAvg)>-3&&(lastBeatAvg-beatAvg)<3&&(lastBeatAvg-beatAvg)>-3){
            softSerial1.print("心率已经稳定，可以读取心率数据。推荐心率读数: ");
            softSerial1.println(beatAvg);
            rate_validation_flag = 1;
            if(beatAvg<55){
              softSerial1.println("系统检测到您可能存在窦性心率过缓。");
            }else if(beatAvg>90){
              softSerial1.println("系统检测到您可能存在窦性心率过速。");
            }
          }    
        }

      }
    }
    if(i%200==0){
      softSerial1.print("瞬时心率=");
      softSerial1.print(beatsPerMinute);
      softSerial1.print(", 平均心率=");
      softSerial1.print(beatAvg);
    
      if (irValue < 50000){
        softSerial1.print(" 未检测到手指。");
        rate_validation_flag = 0;
      }
    
      softSerial1.println();
    }
    i++;
  }else if(mode==2){
    for(int i=0;i<t_size;i++){
      delay(100);
      ta += mlx.readAmbientTempC();
      tb += mlx.readObjectTempC();
    }
    ta /= t_size;
    tb /= t_size;
    //利用标准温度曲线进行校正
    ta = 0.9273*ta+1.4974;
    tb = 0.9273*tb+1.4974;
    if((tb-last_tb)<0.1&&(tb-last_tb)>-0.1&&tb>35&&temperature_validation_flag==0){
      softSerial1.print("指尖温度已稳定，推荐温度读数为:"); softSerial1.print(tb); softSerial1.println("*C");
      if(tb<35){
        softSerial1.print("指尖温度过低，请稍后测量。");
      }
      if(tb>35&&tb<37.1){
        softSerial1.print("指尖温度正常，请放心。");
      }
      if(tb>37.1){
        softSerial1.print("指尖温度过高，疑似高温症状，注意手指不要施压。");
      }
      temperature_validation_flag = 1;
    }
    last_tb=tb;
    softSerial1.print("环境温度 = "); softSerial1.print(ta); 
    softSerial1.print("*C\t指尖温度 = "); softSerial1.print(tb); softSerial1.println("*C");
    if(tb<30){
      softSerial1.print("未检测到手指。");
      temperature_validation_flag = 0;
    }
    softSerial1.println();
  }else if(mode==3){
    Serial.println(particleSensor.getIR());
  }else if(mode==4){
  }

  
  //read softSerial data
  if(softSerial1.available()){
    message = softSerial1.read()-'0';
    if(message==1){
      if(mode!=1){
        mode = 1;
        beep();
        softSerial1.println("设备将切换为[心率模式]");
        rate_validation_flag = 0;
        digitalWrite(heart_rate_power, HIGH);
        digitalWrite(temperature_power, LOW);
        softSerial1.println("请将手指放入松紧带。");
        particleSensor.setup(); //Configure sensor with default settings
        particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
        particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
      }
    }
    if(message==2){
      if(mode!=2){
        mode = 2;
        beep();
        softSerial1.println("设备将切换为[温度模式]");
        softSerial1.println("请将食指轻放于温度检测器上。");
        temperature_validation_flag = 0;
        digitalWrite(heart_rate_power, LOW);
        digitalWrite(temperature_power, HIGH);
        mlx.begin();  
      }
    }
    if(message==3){
      if(mode!=3){
        mode = 3;
        beep();
        softSerial1.println("设备将切换为[脉搏波模式]，请在电脑端查看脉搏波波形。");
        digitalWrite(heart_rate_power, HIGH);
        digitalWrite(temperature_power, LOW); 
        softSerial1.println("请将手指放入松紧带。");
        particleSensor.setup(); //Configure sensor with default settings
        particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
        particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
      }
    }
    if(message==4){
      if(mode!=4){
        mode = 4;
        beep();
        softSerial1.println("感谢您的使用，设备将进入待机状态。");
        digitalWrite(heart_rate_power, LOW);
        digitalWrite(temperature_power, LOW);
        digitalWrite(LED_red, LOW);
        digitalWrite(LED_blue, LOW);
        digitalWrite(LED_green, LOW);
        digitalWrite(buzzer, HIGH);
      }
    }
    if(message==5){
      LED_mode ^= 1;
      beep();
    }
    if(message==6){
      beep_mode ^= 1;
      beep();
    }
    if(message==7){
      beep();
      resetFunc();
    }
  }
  
}
