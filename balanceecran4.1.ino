/* 
 *  Programme Balance Bluetooth Arduino
 *  Author: M.Hameau
 *  2015-2018
 */
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
#include <SoftwareSerial.h>
#endif

#define MATBLE 1

#ifdef MATBLE
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#endif
//#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

//serial debug on/off
#define MSGDEBUG 0

/* ****************  BLE BLUETOOTH MODULE *************************************/
#ifdef MATBLE
//Perform a factory reset when running this sketch
#define FACTORYRESET_ENABLE         1
//Minimum firmware version to have some new features
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"

// Create the bluefruit object, either software serial...uncomment these lines
/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error() {
  while (1);
}

/* ****************  GATT service */
#include "Adafruit_BLEGatt.h"
//init service GATT
Adafruit_BLEGatt gatt(ble);
int32_t poidsCharacteristicCurr;

//BLE SETUP FUNCTION
void setup_ble()
{
  /* Initialise the module */
#ifdef MSGDEBUG
  Serial.print(F("Initialising the Bluefruit LE module: "));
#endif
  if ( !ble.begin(VERBOSE_MODE) )
  {
    error();//Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
#ifdef MSGDEBUG
  Serial.println( F("OK!") );
#endif

  if ( FACTORYRESET_ENABLE )
  {
    // Perform a factory reset to make sure everything is in a known state 
    if ( ! ble.factoryReset() )
    {
      error();//Couldn't factory reset"));
    }
  }
  // Disable command echo from Bluefruit 
  ble.echo(false);

#ifdef MSGDEBUG
  // Print Bluefruit information 
  ble.info();
#endif
  //ble.sendCommandCheckOK("AT+BLEPOWERLEVEL=-16");
  // debug info is a little annoying after this point!
  ble.verbose(false);  

  //setup GATT value 
  gatt.addService(0x1809);
  /* characteristics, current weigth */
  poidsCharacteristicCurr = gatt.addCharacteristic(0x2221,
                                                     GATT_CHARS_PROPERTIES_READ | GATT_CHARS_PROPERTIES_NOTIFY,
                                                     sizeof(float), sizeof(float), BLE_DATATYPE_BYTEARRAY);
}
#endif
/* ****************  BALANCE HX711 *************************************/
#include "Hx711.h"
#define SCK_PIN   6
#define DT_PIN    12
#define NB_PESEE  30
/* initiate scale... */
Hx711 scale(DT_PIN, SCK_PIN);

/* ****************  BATTERY CHECK *************************************/
//pin pour check voltage batterie
#define VBATPIN   A9
//vars pour VMAX et VMIN
#define BATT_VCHARGE 4.59 //4.3
#define BATT_VMAX 4.2 //4.18
#define BATT_VMIN 3.70

//var globale pour la derniere mesure (moyenne)
float lastBattVal = 4.20;
//Affiche le voltage de la batterie lu
float getBattVal() {
  /*float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  return measuredvbat;*/
  return ((analogRead(VBATPIN) * 6.6) / 1024);
}

//convertit le voltage en pourcentage
int getBattPercent() {
  int bperc = 0;
  //recupere le voltage et fait la moyenne
  lastBattVal = (getBattVal()+ lastBattVal) / 2;
  //if we are charging, return a special value
  if (lastBattVal > BATT_VCHARGE)
    return 200;
  //calcul du pourcentage
  bperc = (100 * (lastBattVal - BATT_VMIN)) / (BATT_VMAX - BATT_VMIN);
  if (bperc > 100)
    return 100;
  if (bperc < 0)
    return 0;
  return bperc;
}

/* ****************  SWITCHes *************************************/
#define MY_PIN_LEFT   10 
#define MY_PIN_RIGHT   5

//setup both switch
void setup_switch()
{
  // button input settings
  //************************
  pinMode(MY_PIN_LEFT,INPUT_PULLUP);
  pinMode(MY_PIN_RIGHT,INPUT_PULLUP);
}

/* ****************  OLED SCREEN *************************************/
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 0
#define OLED_I2C_ADDR 0x3C
Adafruit_SSD1306 display(OLED_RESET);

//#if (SSD1306_LCDHEIGHT != 64)
//#error("");//Height incorrect, please fix Adafruit_SSD1306.h!");
//#endif

//to toggle screen ON/OFF
int g_toggleScreen = 0;

//historique
#define HISTO_HAUT 24
#define HISTO_LEN 128
#define HISTO_SHOTW_DEFAULT 20
unsigned char gHISTO_SHOTW_MAX = HISTO_SHOTW_DEFAULT;
unsigned char historique[HISTO_LEN];

//Icons
static const unsigned char PROGMEM logoBluetooth[] =
{ 0x0E, 0x70, 0x1E, 0x38, 0x36, 0x9C, 0x32, 0xCC, 0x78, 0xE6, 0x7C, 0xCE, 0x7E, 0x9E, 0x7E, 0x3E,
  0x7E, 0x9E, 0x7C, 0xCE, 0x78, 0xE6, 0x72, 0xCE, 0x36, 0x9C, 0x3E, 0x3C, 0x1E, 0x78, 0x0F, 0xF0
};
static const unsigned char PROGMEM logoBattery[] =
{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xE0, 0x3F, 0xFF, 0xF0, 0x30,
  0x00, 0x30, 0x30, 0x00, 0x3C, 0x30, 0x00, 0x3C, 0x30, 0x00, 0x3C, 0x30, 0x00, 0x3C, 0x30, 0x00,
  0x30, 0x3F, 0xFF, 0xF0, 0x1F, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
static const unsigned char PROGMEM logoBattcharge[] = {
  0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x0C, 0x00, 0x1F, 0x99, 0xE0, 0x3F, 0x3B, 0xF0, 0x30,
  0x70, 0x30, 0x30, 0xF0, 0x3C, 0x31, 0xFE, 0x3C, 0x33, 0xFC, 0x3C, 0x30, 0x78, 0x3C, 0x30, 0x70,
  0x30, 0x3E, 0xE7, 0xF0, 0x1C, 0xCF, 0xE0, 0x01, 0x80, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00
};
//initialize Oled screen
void setup_display()
{
    //init screen and show Adafruit logo
  //**********************************
  display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR, false);  // initialize with the I2C addr (for the 128x64)
  display.display();
  delay(2000);
  display.clearDisplay();
  display.display();
}

/* ****************  SETUP *************************************/
void setup(void)
{
  //*** uncomment to wait for the serial debug console ***
#ifdef MSGDEBUG
//  while (!Serial);  // required for Flora & Micro
//  delay(500);
  
  Serial.begin(115200);
//  Serial.println(F("Adafruit Bluefruit --- Balance Mode by Matt"));
//  Serial.println(F("-------------------------------------------"));
#endif

  // ble module init
  //************************  
#ifdef MATBLE
  setup_ble();
#endif
  // screen init
  //************************  
  setup_display();
  // button init
  //************************  
  setup_switch();
  // scale setting
  //***********************
  scale.setScale(900);//901);
  //scale.setOffset(scale.averageValue());
}

/* ****************  LOOP FUNCTION *************************************/
unsigned int old_timestamp = millis();
unsigned int chrono_start = old_timestamp;
unsigned int chrono_stop = old_timestamp;
char chrono_started = 0;
float old_fpoids = 0;

void loop(void)
{
  int i;
  unsigned int loopstart = millis();
  int bval = getBattPercent();
  union float_bytes {
    float value;
    uint8_t bytes[sizeof(float)];
  };
  static union float_bytes tPoids = { .value = 0.0 };

  //poids arrondi à 0.1
  tPoids.value = floor(10*scale.getGram2(NB_PESEE)+0.5)/10;
  
  //filtre pour éviter la variation de poids
  if(abs(tPoids.value - old_fpoids) <= 1)
    tPoids.value = (tPoids.value + old_fpoids+ old_fpoids) / 3;
  //pour éviter le -0.0
  if((tPoids.value < 0) && (tPoids.value > -0.1))
    tPoids.value = 0.0;  

#ifdef MATBLE
  //update du poids via bluetooth gatt
  gatt.setChar(poidsCharacteristicCurr, tPoids.bytes, sizeof(tPoids));
#endif
  
  // Button state
  if ((digitalRead(MY_PIN_LEFT) == LOW) || (digitalRead(MY_PIN_RIGHT) == LOW))
  {
      int i;
      //display message
      display.clearDisplay();
      display.setCursor(2, 0);
      display.setTextSize(3);
      display.print("RAZ:"); display.display();
      //RAZ historique
      for(i=0; i< HISTO_LEN; i++)
        historique[i] = 0;
      gHISTO_SHOTW_MAX = HISTO_SHOTW_DEFAULT;
      //RAZ chrono
      chrono_start = chrono_stop;
      //attente 1 seconde pour ne pas influencer la mesure
      delay(1000);
      //pesee + tare
      tPoids.value = scale.averageValue(NB_PESEE);
      scale.setOffset(tPoids.value);
      //display.setCursor(2, 5);
      display.print("OK"); display.display();
  }
  //other button
/*  else if (digitalRead(MY_PIN_RIGHT) == LOW)
  {
      //display message
      display.clearDisplay();
      display.setCursor(2, 0);
      display.setTextSize(3);
      display.print("TARGET:"); 
      display.display();
      
  }*/
  //else, update screen
  else{ 
      //efface l'ecran
      display.clearDisplay();

      //affichage de l'historique 
      for (i = 0; i < (HISTO_LEN-1); i++)
      { 
        //convert values to pixels
        unsigned char p = (historique[i] * HISTO_HAUT) / gHISTO_SHOTW_MAX;
        if (p > HISTO_HAUT){
          gHISTO_SHOTW_MAX = historique[i];
          p = HISTO_HAUT;
        }
        display.drawLine(i, 64, i, 64 - p, WHITE);
      }
    
      //shift de l'historique du flow
      float flow = ((tPoids.value - old_fpoids) * 1000) / (loopstart - old_timestamp);
      for (i = 0; i < 127; i++)
        historique[i] = historique[i + 1];
      //update historique poids
      if(tPoids.value > 0)
        if(tPoids.value > 255)
          historique[127] = 255;
        else
          historique[127] = tPoids.value;
      else
        historique[127] = 0;
    
      //démarrage du chrono
      if ((flow > 0.2)/*((tPoids.value - old_fpoids)>1)*/ && !chrono_started)
      {
        //restart only after 3 seconds inactivity
        if ((loopstart - chrono_stop) > 3000)
          chrono_start = loopstart;
        chrono_started = 1;
      }
      //stop du chrono
      if ((flow < 0.2)/*((tPoids.value - old_fpoids)<1)*/ && chrono_started)
      {
        chrono_stop = loopstart;
        chrono_started = 0;
      }
    
      //affichage du texte
      display.setTextColor(WHITE);
      display.setCursor(1, 0);
      //affiche le chrono
      display.setTextSize(2);
      display.print("t");
      {
        int sec, ms;
        if (chrono_started)
        {
          sec = (int)((loopstart - chrono_start) / 1000);
          ms = (loopstart - chrono_start) - (sec * 1000);
        } else {
          sec = (int)((chrono_stop - chrono_start) / 1000);
          ms = (chrono_stop - chrono_start) - (sec * 1000);
        }
        display.print(sec);
        display.print(".");
        display.print(ms/100);
        display.println("s");
      }
      //affiche le poids en gros
      display.setTextSize(3);
      display.print(String(tPoids.value, 1));
      display.println("g");
      //affiche le flow
      display.setTextSize(1);
      display.print("Fl:");
      display.print(String(flow, 1));
      display.println("g/s");
      //display.setTextSize(1);
      //affiche le voltage batterie
      display.print("Bt:");
      display.print(getBattVal());
      display.println("v");
   
#ifdef MATBLE
      //affiche l'icone BT
      if (ble.isConnected()) {
        display.drawBitmap(80, 0, logoBluetooth, 16, 16, WHITE);
      }
#endif
      //Gauge de batterie
      if (bval < 200) {
        //disp batt icon
        display.drawBitmap(104, 0, logoBattery, 24, 16, WHITE);
        bval = (bval / 8);
        if (bval > 1) {
          display.fillRect(109, 7, bval, 3, WHITE);
        }
      } else {
        //disp batt charge icon
        display.drawBitmap(104, 0, logoBattcharge, 24, 16, WHITE);
      }
      
      // Update screen
      display.display();
  }  
  /*
  if (digitalRead(MY_PIN_RIGHT) == LOW)
  {

    if(g_toggleScreen){
      display.ssd1306_command(SSD1306_DISPLAYON); // To switch display back on
      g_toggleScreen = 0;
      //print message
      display.clearDisplay();
      display.setCursor(2, 0);
      display.setTextSize(3);
      display.print("SCREEN ON       "); display.display();     
      delay(500);

    }else{
      //print message
      display.clearDisplay();
      display.setCursor(2, 0);
      display.setTextSize(3);
      display.print("SCREEN OFF       "); display.display();     
      delay(500);
      display.ssd1306_command(SSD1306_DISPLAYOFF); // To switch display off
      g_toggleScreen = 1;
    }
  }*/
  
  old_fpoids = tPoids.value;
  old_timestamp = loopstart;
}


