// FOUR AXIS CHANNELS

// Y1 > CH1 Throttle
// X1 > CH2 Rudder (yaw)
// Y2 > CH3 Elevator (pitch)
// X2 > CH4 Aileron (roll)


//#include <EEPROM.h>
#include "U8glib.h" //Install U8glib Library to be able to compile
#include <nRF24L01.h>
#include <RF24.h>


U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0 | U8G_I2C_OPT_NO_ACK | U8G_I2C_OPT_FAST);  // Fast I2C / TWI
//U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NO_ACK);  // Display which does not send ACK
//U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NONE);  // I2C / TWI

//Hardware setup
#define CSN             10 //RF-Nano 10
#define CE              9 //RF-Nano 9

#define X1_in       A0
#define Y1_in       A1

#define X2_in       A2
#define Y2_in       A3

#define RIGHT_BT    5
#define LEFT_BT     6

#define VSENS       A6


//Settings setup

int X1_value = 0;
int X1_center = 510;

int Y1_value = 0;
int Y1_center = 528;

int X2_value = 0;
int X2_center = 492;

int Y2_value = 0;
int Y2_center = 505;

int CH1_output = 1000;
int CH2_output = 1000;
int CH3_output = 1000;
int CH4_output = 1000;
int CH5_output = 1000;
int CH6_output = 1000;

long previousMillis = 0;
int vsens;
float voltage;

int expo = 0; //3-5 is a good value, set to 0 to deactivate
int deadband = 3;

byte pressedbut = 0;
byte i_butt = 0;

const uint64_t my_radio_pipe = 0xE8E8F0F0E1LL;     //Remember that this code is the same as in the transmitter
RF24 radio(CSN, CE);  //CSN and CE pins

struct Data_to_be_sent {
  uint16_t ch1;
  uint16_t ch2;
  uint16_t ch3;
  uint16_t ch4;
  uint16_t ch5;
  uint16_t ch6;
};

//Create a variable with the structure above and name it sent_data
Data_to_be_sent sent_data;


/**********************************************************/

void setup() {

  analogReference(DEFAULT);

  radio.begin();
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_MAX); //RF24_PA_MIN -18dBm, RF24_PA_LOW -12dBm, RF24_PA_HIGH -6dBM and RF24_PA_MAX 0dBm
  radio.setDataRate(RF24_250KBPS);
  //radio.setChannel(108); // 2.508 Ghz - Above most Wifi Channels
  radio.openWritingPipe(my_radio_pipe);

  //Reset each channel value
  sent_data.ch1 = 1500;
  sent_data.ch2 = 1500;
  sent_data.ch3 = 1500;
  sent_data.ch4 = 1500;
  sent_data.ch5 = 1000;
  sent_data.ch6 = 1000;

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


  pinMode(X1_in, INPUT);
  pinMode(Y1_in, INPUT);

  pinMode(X2_in, INPUT);
  pinMode(Y2_in, INPUT);

  pinMode(VSENS, INPUT);

  pinMode(LEFT_BT, INPUT_PULLUP);
  pinMode(RIGHT_BT, INPUT_PULLUP);

  //Serial.begin(9600);

  clearOLED();

  analogRead(X1_in);
  X1_center = analogRead(X1_in);
  analogRead(X2_in);
  X2_center = analogRead(X2_in);
  analogRead(Y1_in);
  Y1_center = analogRead(Y1_in);
  analogRead(Y2_in);
  Y2_center = analogRead(Y2_in);

}


/**********************************************************/


void clearOLED()
{
  u8g.firstPage();
  do
  {
  }
  while ( u8g.nextPage() );
}


/**********************************************************/


byte buttoncheck()
{
  int i_butt = 0;
  byte buttonz = 0;
  if (digitalRead(RIGHT_BT) != 1)
  {
    while (digitalRead(RIGHT_BT) != 1)
    {
      delay(2);
      i_butt++;
    }
    buttonz = 2;
  }
  if (digitalRead(LEFT_BT) != 1)
  {
    while (digitalRead(LEFT_BT) != 1)
    {
      delay(2);
      i_butt++;
    }
    buttonz = 1;
  }

  pressedbut = buttonz;
  return buttonz;
}


/**********************************************************/


void loop() {
  // 1000 reverse, 1500 stop, 2000 forward
  readinputs();
  buttoncheck();
  infoscreen();
  readvoltage();


  if (pressedbut == 1) { //BT LEFT
    if (CH5_output == 1000) {
      CH5_output = 1500;
    }
    else if (CH5_output == 1500) {
      CH5_output = 2000;
    }
    else {
      CH5_output = 1000;
    }

  }


  if (pressedbut == 2) { //BT RIGHT
    if (CH6_output == 1000) {
      CH6_output = 1500;
    }
    else if (CH6_output == 1500) {
      CH6_output = 2000;
    }
    else {
      CH6_output = 1000;
    }
  }


  sent_data.ch1 = CH1_output;
  sent_data.ch2 = CH2_output;
  sent_data.ch3 = CH3_output;
  sent_data.ch4 = CH4_output;
  sent_data.ch5 = CH5_output; //LEFT BT
  sent_data.ch6 = CH6_output; //RIGHT BT

  radio.write(&sent_data, sizeof(Data_to_be_sent));

  //delay(20);

}

// USAGE
//smoothIt(from, to, value, power, reverse);
//from: where does the curve start? Normally 0
//to: where does the curve stop?
//value: the actual value you want to have calculated and returned
//power: the power of smoothness. 1 is flat. A good value is from 2 to 5
//reverse: reverses the curve (0/1)


int smoothIt(int from, int to, int val, int power, int reverse) {
  float to2;
  to2 = to - from;
  int ret;
  if (reverse == 1) {
    ret = (pow((val - from) / to2 - 1, power) + 1) * to2 + from; //
    return ret;
  } else {
    ret = pow((val - from) / to2, power) * to2 + from; //
    return ret;
  }
}


/**********************************************************/


void readinputs() {

  //smoothing by taking the average of four measurments
  volatile int X1_sum = 0;
  X1_sum = analogRead(X1_in);
  X1_sum += analogRead(X1_in);
  X1_sum += analogRead(X1_in);
  X1_sum += analogRead(X1_in);
  X1_sum = X1_sum / 4;
  int X1_newvalue = X1_sum;

  volatile int Y1_sum = 0;
  Y1_sum = analogRead(Y1_in);
  Y1_sum += analogRead(Y1_in);
  Y1_sum += analogRead(Y1_in);
  Y1_sum += analogRead(Y1_in);
  Y1_sum = Y1_sum / 4;
  int Y1_newvalue = Y1_sum;

  volatile int X2_sum = 0;
  X2_sum = analogRead(X2_in);
  X2_sum += analogRead(X2_in);
  X2_sum += analogRead(X2_in);
  X2_sum += analogRead(X2_in);
  X2_sum = X2_sum / 4;
  int X2_newvalue = X2_sum;

  volatile int Y2_sum = 0;
  Y2_sum = analogRead(Y2_in);
  Y2_sum += analogRead(Y2_in);
  Y2_sum += analogRead(Y2_in);
  Y2_sum += analogRead(Y2_in);
  Y2_sum = Y2_sum / 4;
  int Y2_newvalue = Y2_sum;


  if (X1_newvalue != X1_value) {
    X1_value = X1_newvalue;

    if (abs(X1_newvalue - X1_center) < deadband) {
      CH2_output = 1500;
    }

    else if (X1_newvalue < X1_center) { //move to right 512(CENTER) -> 0
      if (expo > 0) {
        X1_value = smoothIt( 0, X1_center, X1_value, expo, 0);
      }
      CH2_output = map(X1_value, 0, X1_center, 2000, 1500);
    }

    else if (X1_newvalue > X1_center) { //move to left 1024 <- 512(CENTER)
      if (expo > 0) {
        X1_value = smoothIt(X1_center, 1023, X1_value, expo, 1);
      }
      CH2_output = map(X1_value, X1_center, 1023, 1500, 1000);
    }

  }


  if (Y1_newvalue != Y1_value) {
    Y1_value = Y1_newvalue;

    if (abs(Y1_newvalue - Y1_center) < deadband) {
      CH1_output = 1500;
    }

    else if (Y1_newvalue < Y1_center) { //move DOWN 512(CENTER) -> 0
      if (expo > 0) {
        Y1_value = smoothIt( 0, Y1_center, Y1_value, expo, 1);
      }
      CH1_output = map(Y1_value, 0, Y1_center, 1000, 1500);
    }

    else if (Y1_newvalue > Y1_center) { //move UP 1024 <- 512(CENTER)
      if (expo > 0) {
        Y1_value = smoothIt(Y1_center, 1023, Y1_value, expo, 0);
      }
      CH1_output = map(Y1_value, Y1_center, 1023, 1500, 2000);
    }

  }

  //Serial.print("Y1 NEW: ");
  //Serial.println(Y1_newvalue); //get the summed measurment
  //Serial.print("Y1 SM: ");
  //Serial.println(Y1_value); //get the summed measurment


  if (X2_newvalue != X2_value) {
    X2_value = X2_newvalue;

    if (abs(X2_newvalue - X2_center) < deadband) {
      CH4_output = 1500;
    }

    else if (X2_newvalue < X2_center) {
      //mapping the output 1000-2000 when the delta is more than the DEADBAND
      CH4_output = map(X2_value, 0, X2_center, 1000, 1500);
    }

    else if (X2_newvalue > X2_center) {
      //mapping the output 1000-2000 when the delta is more than the DEADBAND
      CH4_output = map(X2_value, X2_center, 1023, 1500, 2000);
    }

  }


  if (Y2_newvalue != Y2_value) {
    Y2_value = Y2_newvalue;

    if (abs(Y2_newvalue - Y2_center) < deadband) {
      CH3_output = 1500;
    }

    else if (Y2_newvalue < Y2_center) {
      //mapping the output 1000-2000 when the delta is more than the DEADBAND
      CH3_output = map(Y2_value, 0, Y2_center, 2000, 1500);
    }

    else if (Y2_newvalue > Y2_center) {
      //mapping the output 1000-2000 when the delta is more than the DEADBAND
      CH3_output = map(Y2_value, Y2_center, 1023, 1500, 1000);
    }

  }
  
}


/**********************************************************/


void infoscreen()
{
  u8g.firstPage();
  do
  {
    u8g.setPrintPos(0, 10);
    u8g.setFont(u8g_font_6x10r);
    u8g.print("X01");
    u8g.drawFrame (20, 2, 40, 8);
    int bar1 = map(CH2_output, 1000, 2000, 0, 36);
    u8g.drawBox (22, 4, bar1, 4);

    u8g.setPrintPos(0, 20);
    u8g.print("Y01");
    u8g.drawFrame (20, 12, 40, 8);
    int bar2 = map(CH1_output, 1000, 2000, 0, 36);
    u8g.drawBox (22, 14, bar2, 4);

    u8g.setPrintPos(66, 10);
    u8g.print("X02");
    u8g.drawFrame (86, 2, 40, 8);
    int bar4 = map(CH4_output, 1000, 2000, 0, 36);
    u8g.drawBox (88, 4, bar4, 4);

    u8g.setPrintPos(66, 20);
    u8g.print("Y02");
    u8g.drawFrame (86, 12, 40, 8);
    int bar3 = map(CH3_output, 1000, 2000, 0, 36);
    u8g.drawBox (88, 14, bar3, 4);


    //STICK ANIMATION LEFT
    u8g.drawFrame (9, 30, 33, 33);
    int pos_y1 = map(CH1_output, 1000, 2000, 59, 33);
    int pos_x1 = map(CH2_output, 1000, 2000, 12, 38);
    u8g.drawDisc (pos_x1, pos_y1, 2);

    int bar_bt_l = map(CH5_output, 1000, 2000, 1, 33);
    u8g.drawBox (3, 63-bar_bt_l, 3, bar_bt_l);
    
    if (pressedbut == 1) {
      u8g.drawBox (10, 30, 33, 33);
    }


    //STICK ANIMATION RIGHT
    u8g.drawFrame (86, 30, 33, 33);
    int pos_y2 = map(CH3_output, 1000, 2000, 59, 33);
    int pos_x2 = map(CH4_output, 1000, 2000, 89, 115);
    u8g.drawDisc (pos_x2, pos_y2, 2);

    int bar_bt_r = map(CH6_output, 1000, 2000, 1, 33);
    u8g.drawBox (122, 63-bar_bt_r, 3, bar_bt_r);

    if (pressedbut == 2) {
      u8g.drawBox (86, 30, 33, 33);
    }

    if (voltage < 3) {
      u8g.setPrintPos(56, 58);
      u8g.print("USB");
    }

    else {
      u8g.setPrintPos(53, 58);
      u8g.print(voltage, 1);
      u8g.print("v");
    }

    //BAT ICON
    u8g.drawFrame (54, 35, 23, 10);
    u8g.drawBox (52, 38, 2, 4);

    if ((voltage <= 4.29) && (voltage > 4.0))  {
      u8g.drawBox (56, 37, 4, 6); //1st bar 25%
      u8g.drawBox (61, 37, 4, 6); //2nd bar 50%
      u8g.drawBox (66, 37, 4, 6); //3rd bar 75%
      u8g.drawBox (71, 37, 4, 6); //4th bar 100%
    }
    else if ((voltage <= 4.0) && (voltage > 3.8)) {
      u8g.drawBox (56, 37, 4, 6); //1st bar 25%
      u8g.drawBox (61, 37, 4, 6); //2nd bar 50%
      u8g.drawBox (66, 37, 4, 6); //3rd bar 75%
    }
    else if ((voltage <= 3.8) && (voltage > 3.6)) {
      u8g.drawBox (56, 37, 4, 6); //1st bar 25%
      u8g.drawBox (61, 37, 4, 6); //2nd bar 50%
    }
    else if ((voltage <= 3.6)  && (voltage > 3.3)) {
      u8g.drawBox (56, 37, 4, 6); //1st bar 25%
    }
    else {
    }


  }
  while ( u8g.nextPage() );
}


/**********************************************************/


void readvoltage(void) {

  vsens = analogRead(VSENS);

  unsigned long updatetime = millis();

  if (updatetime - previousMillis > 1500) { // update voltage reading after one second
    previousMillis = updatetime;

    voltage = vsens * (4.12 / 1023.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage, considering the voltage divider:
  }
}
