/*
  NRF24 Module > Arduino NANO
  GND    ->   GND
  Vcc    ->   3.3V
  CE     ->   D9
  CSN    ->   D10
  CLK    ->   D13
  MOSI   ->   D11
  MISO   ->   D12

  HG7881 Connection:
  1A should be used for PWM and 1B for direction in the HG7881
  Motor FrontRight > D8 DIR | D6 PWM speed
  Motor RearLeft > D7 DIR | D5 PWM speed

*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "U8glib.h" //Install U8glib Library to be able to compile
#include <Adafruit_NeoPixel.h>

//Hardware setup
#define CSN             9
#define CE              10

#define FR_dir          8
#define FR_speed        6
#define RL_dir          7
#define RL_speed        5

#define LED_pin         4

#define trigPin         3
#define echoPin         2

#define NUMPIXELS       2

//U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0 | U8G_I2C_OPT_NO_ACK | U8G_I2C_OPT_FAST);  // Fast I2C / TWI
U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_DEV_0 | U8G_I2C_OPT_FAST); // Dev 0, Fast I2C / TWI

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, LED_pin, NEO_GRB + NEO_KHZ800);

const uint64_t pipeIn = 0xE8E8F0F0E1LL; //Remember that this code is the same as in the transmitter
RF24 radio(CSN, CE);  //CSN(9) and CE(10) pins

struct Received_data {
  uint16_t ch1;
  uint16_t ch2;
  uint16_t ch3;
  uint16_t ch4;
  uint16_t ch5;
  uint16_t ch6;
};

Received_data received_data;

int ch1_value = 0;
int ch2_value = 0;
int ch3_value = 0;
int ch4_value = 0;
int ch5_value = 0;
int ch6_value = 0;

byte failsafe = 1;
int distancecm = 0;
int bank_value = 0;
int steer_value = 1;
int speed_value = 0;

byte lightmode = 0;
int animationtimer = 0;
byte animationstep = 0;
byte movestep = 0;
byte obstical_flag = 0;
unsigned long previousMillis = 0;
unsigned long previousMoveMillis = 0;

//sonar values
long duration;
int distance;
int new_distance;
int obstacle = 0;

void reset_the_Data()
{
  // 'safe' values to use when NO radio input is detected
  received_data.ch1 = 1500;      //Throttle (channel 1) to 0
  received_data.ch2 = 1500;
  received_data.ch3 = 1500;
  received_data.ch4 = 1500;
  received_data.ch5 = 1000;
  received_data.ch6 = 1000;
}


/**********************************************************/


void setup()
{
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  pinMode(FR_dir, OUTPUT);
  pinMode(FR_speed, OUTPUT);
  pinMode(RL_dir, OUTPUT);
  pinMode(RL_speed, OUTPUT);

  //stop motors
  digitalWrite(RL_speed, LOW);
  digitalWrite(RL_dir, LOW);
  digitalWrite(FR_speed, LOW);
  digitalWrite(FR_dir, LOW);

  // Serial.begin(9600);

  //We reset the received values
  reset_the_Data();

  //Once again, begin and radio configuration
  radio.begin();
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_MAX); //RF24_PA_MIN -18dBm, RF24_PA_LOW -12dBm, RF24_PA_HIGH -6dBM and RF24_PA_MAX 0dBm
  radio.setDataRate(RF24_250KBPS);
  //radio.setChannel(108); // 2.508 Ghz - Above most Wifi Channels
  radio.openReadingPipe(1, pipeIn);

  //We start the radio comunication
  radio.startListening();

  if ( u8g.getMode() == U8G_MODE_R3G3B2 ) {
    u8g.setColorIndex(255);     // white
  }
  else if ( u8g.getMode() == U8G_MODE_GRAY2BIT ) {
    u8g.setColorIndex(3);         // max intensity
  }
  else if ( u8g.getMode() == U8G_MODE_BW ) {
    u8g.setColorIndex(1);         // pixel on
  }
  else if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
    u8g.setHiColorByRGB(255, 255, 255);
  }

  pixels.begin(); // This initializes the NeoPixel library.

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
}


/**********************************************************/


unsigned long lastRecvTime = 0;

//We create the function that will read the data each certain time
void receive_the_data()
{
  while ( radio.available() ) {
    radio.read(&received_data, sizeof(Received_data));
    lastRecvTime = millis(); //Here we receive the data
  }
}


/**********************************************************/


void loop()
{
  receive_the_data();

  //////////This will reset the data if signal is lost for 1 sec.
  unsigned long now = millis();
  if ( now - lastRecvTime > 1000 ) {
    // signal lost?
    reset_the_Data();
    failsafe = 1;
  }
  else {
    failsafe = 0;
  }

  ch1_value = received_data.ch1;
  ch2_value = received_data.ch2;
  ch3_value = received_data.ch3;
  ch4_value = received_data.ch4;
  ch5_value = received_data.ch5;
  ch6_value = received_data.ch6;

  sonar(); //ckeck sonar an get distance

  if (ch5_value == 1000) {
    rc_control(); //rc motor control
  }
  else {
    auto_control();
  }

  LED_status(); //LED indication
  OLED(); //OLED display update
}


/**********************************************************/


void sonar() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  if (distance < 100) { //update distance only if in valid range
    new_distance = distance;
  }

  //report an obstacle
  if ((new_distance < 5) && (new_distance > 0)) {
    obstacle = 1;
  }
  else {
    obstacle = 0;
  }
}


/**********************************************************/


void LED_status() {
  // WSLED CODE

  if (ch6_value == 2000) {
    lightmode = 1;
  }

  else if (ch5_value != 1000) {
    lightmode = 2;
  }
  else {
    lightmode = 0;
  }


  if (failsafe == 0) {

    if (obstacle == 1) {
      pixels.setPixelColor(0, pixels.Color(150, 150, 0)); // Moderately bright yellow color.
      pixels.setPixelColor(1, pixels.Color(150, 150, 0)); // Moderately bright yellow color.

      pixels.show(); // This sends the updated pixel color to the hardware.
    }
    else {
      if (lightmode == 0) {
        pixels.setPixelColor(0, pixels.Color(0, 0, 0)); // Moderately bright green color.
        pixels.setPixelColor(1, pixels.Color(0, 0, 0)); // Moderately bright green color.
        pixels.show(); // This sends the updated pixel color to the hardware.
      }

      else if (lightmode == 1) {
        unsigned long animationtimer = millis();

        if (animationtimer - previousMillis >= 60) {
          previousMillis = animationtimer;

          if (animationstep == 0) {
            pixels.setPixelColor(0, pixels.Color(0, 0, 0)); // off
            pixels.setPixelColor(1, pixels.Color(255, 0, 0)); // blue
            animationstep = 1;
          }

          else {
            pixels.setPixelColor(0, pixels.Color(0, 0, 255)); // blue
            pixels.setPixelColor(1, pixels.Color(0, 0, 0)); // off
            animationstep = 0;
          }
          pixels.show(); // This sends the updated pixel color to the hardware.
        }
      }

      else if (lightmode == 2) {
        unsigned long animationtimer = millis();

        if (animationtimer - previousMillis >= 200) {
          previousMillis = animationtimer;

          if (animationstep == 0) {
            pixels.setPixelColor(0, pixels.Color(0, 0, 0)); // off
            pixels.setPixelColor(1, pixels.Color(255, 100, 0)); // blue
            animationstep = 1;
          }

          else {
            pixels.setPixelColor(0, pixels.Color(255, 100, 0)); // blue
            pixels.setPixelColor(1, pixels.Color(0, 0, 0)); // off
            animationstep = 0;
          }
          pixels.show(); // This sends the updated pixel color to the hardware.
        }
      }

    }

  }

  else {
    //FAILSAFE
    pixels.setPixelColor(0, pixels.Color(150, 0, 0)); // Moderately bright red color.
    pixels.setPixelColor(1, pixels.Color(150, 0, 0)); // Moderately bright red color.
    pixels.show(); // This sends the updated pixel color to the hardware.
  }

}


/**********************************************************/


void OLED() {
  // OLED CODE
  u8g.firstPage();
  do
  {
    u8g.setFont(u8g_font_6x10r);

    if (failsafe == 0) {
      u8g.setPrintPos(0, 10);
      u8g.print("Ch1: ");
      u8g.print(ch1_value);
      u8g.drawFrame (60, 2, 60, 8);
      int bar1 = map(ch1_value, 1000, 2000, 0, 57);
      u8g.drawBox (62, 4, bar1, 4);

      u8g.setPrintPos(0, 20);
      u8g.print("Ch2: ");
      u8g.print(ch2_value);
      u8g.drawFrame (60, 12, 60, 8);
      int bar2 = map(ch2_value, 1000, 2000, 0, 57);
      u8g.drawBox (62, 14, bar2, 4);

      u8g.setPrintPos(0, 30);
      u8g.print("Ch3: ");
      u8g.print(ch3_value);
      u8g.drawFrame (60, 22, 60, 8);
      int bar3 = map(ch3_value, 1000, 2000, 0, 57);
      u8g.drawBox (62, 24, bar3, 4);

      u8g.setPrintPos(0, 40);
      u8g.print("Ch4: ");
      u8g.print(ch4_value);
      u8g.drawFrame (60, 32, 60, 8);
      int bar4 = map(ch4_value, 1000, 2000, 0, 57);
      u8g.drawBox (62, 34, bar4, 4);

      u8g.setPrintPos(0, 50);
      u8g.print("movestep ");
      u8g.print(movestep);
      u8g.setPrintPos(0, 60);
      u8g.print("dist ");
      u8g.print(new_distance);
    }


    if (failsafe == 1) {
      u8g.setPrintPos(40, 40);
      u8g.print("NO LINK");
      u8g.setPrintPos(40, 50);
      u8g.print("DIST:");
      u8g.print(distance);
    }

  }
  while ( u8g.nextPage() );

}


/**********************************************************/


void rc_control() {

  //MOTOR control code

  if (failsafe == 0) {
    //FORWARD MOVEMENT
    if (ch1_value > 1510) { //forward
      if (obstacle == 0) {
        speed_value = map(ch1_value, 1510, 2000, 1, 255);
        digitalWrite(RL_dir, HIGH);
        analogWrite(RL_speed, 255 - speed_value + 0); //+0 is trim
        digitalWrite(FR_dir, HIGH);
        analogWrite(FR_speed, 255 - speed_value + 0); //+4 is trim

        //STEER RIGHT while FORWARD
        if (ch4_value > 1510) { //going right
          bank_value = speed_value / 2;
          steer_value = map(ch4_value, 1505, 2000, 0, bank_value);
          analogWrite(FR_speed, (255 - speed_value) + steer_value);
        }

        //STEER LEFT while FORWARD
        else if (ch4_value < 1490) { //going left
          bank_value = speed_value / 2;
          steer_value = map(ch4_value, 1495, 1000, 0, bank_value);
          analogWrite(RL_speed, (255 - speed_value) + steer_value);
        }

        else {
          steer_value = 0;
        }
      }
      else {
        //stop motors
        digitalWrite(RL_speed, LOW);
        digitalWrite(RL_dir, LOW);
        digitalWrite(FR_speed, LOW);
        digitalWrite(FR_dir, LOW);
      }
    }

    //BACKWARD MOVEMENT
    else if (ch1_value < 1490) {
      speed_value = map(ch1_value, 1490, 1000, 1, 255);
      digitalWrite(RL_dir, LOW);
      analogWrite(RL_speed, speed_value);
      digitalWrite(FR_dir, LOW);
      analogWrite(FR_speed, speed_value);

      //STEER RIGHT while BACKWARD
      if (ch4_value > 1510) { //going right
        bank_value = speed_value / 1.5; //trim
        steer_value = map(ch4_value, 1505, 2000, 0, bank_value);
        analogWrite(FR_speed, speed_value - steer_value);
      }

      //STEER LEFT while BACKWARD
      else if (ch4_value < 1490) { //going left
        bank_value = speed_value / 1.7; //trim
        steer_value = map(ch4_value, 1495, 1000, 0, bank_value);
        analogWrite(RL_speed, speed_value - steer_value);
      }

      else {
        steer_value = 0;
      }
    }

    else {
      //stop motors
      digitalWrite(RL_speed, LOW);
      digitalWrite(RL_dir, LOW);
      digitalWrite(FR_speed, LOW);
      digitalWrite(FR_dir, LOW);

      steer_value = 0;
      speed_value = 0;
    }
  }

  if (failsafe == 1) {
    //stop motors on failsafe
    digitalWrite(RL_speed, LOW);
    digitalWrite(RL_dir, LOW);
    digitalWrite(FR_speed, LOW);
    digitalWrite(FR_dir, LOW);
    speed_value = 0;
    steer_value = 0;
  }
}



void auto_control() {

  //MOTOR control code

  if (failsafe == 0) {
    if (distance < 4) {
      
      //stop motors
          digitalWrite(RL_speed, LOW);
          digitalWrite(RL_dir, LOW);
          digitalWrite(FR_speed, LOW);
          digitalWrite(FR_dir, LOW);
      
          
      obstical_flag = 1;
    }

    if (obstical_flag == 1) {
      unsigned long movetimer = millis();

      if (movetimer - previousMoveMillis >= 500) {
        previousMoveMillis = movetimer;

        if (movestep == 0) {
          movestep = 1;
        }
        else if (movestep == 1) {
          movestep = 2;
        }
        else if (movestep == 2) {
          movestep = 3;

        }
        else {
          movestep = 0;
        }
      }

      else {

        if (movestep == 0) {
          //stop motors
          digitalWrite(RL_speed, LOW);
          digitalWrite(RL_dir, LOW);
          digitalWrite(FR_speed, LOW);
          digitalWrite(FR_dir, LOW);
        }

        else if (movestep == 1) {
          //backwards
          digitalWrite(FR_dir, LOW);
          analogWrite(FR_speed, 200);
          digitalWrite(RL_dir, LOW);
          analogWrite(RL_speed, 200);
        }

        else if (movestep == 2) {
          //turn
          digitalWrite(FR_dir, LOW);
          analogWrite(FR_speed, 100);
          digitalWrite(RL_dir, HIGH);
          analogWrite(RL_speed, 100);
        }

        else if (movestep == 3) {
          //reset
          movestep = 0;
          obstical_flag = 0;
        }

      }




    }

    else {
      //move forward
      digitalWrite(RL_dir, HIGH);
      analogWrite(RL_speed, 100);
      digitalWrite(FR_dir, HIGH);
      analogWrite(FR_speed, 100);
      movestep = 0;
    }


  }

  else {
    //stop motors
    digitalWrite(RL_speed, LOW);
    digitalWrite(RL_dir, LOW);
    digitalWrite(FR_speed, LOW);
    digitalWrite(FR_dir, LOW);
  }


}
