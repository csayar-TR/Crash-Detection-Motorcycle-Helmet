// ı2c lcd ekran ve şarj kontrolü
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,16,2); // Bu kodu kullanırken ekranda yazı çıkmaz ise 0x27 yerine 0x3f yazınız !!
int pinSarj25 = 17 ;
int pinSarj50 = 5 ;
int pinSarj75 = 18 ;
int pinSarj100 = 19 ;

bool sarj25 = 0 ;
bool sarj50 = 0 ;
bool sarj75 = 0 ;
bool sarj100 = 0 ;
bool uyari = true ;
//Kanama Sensörü ************* Gerekli Tanımlamalar
int rainPin = 35;
bool kanama = false ;
// you can adjust the threshold value
int thresholdValue = 2200;
//Gyroscope Sensor************ Gerekli Tanımlamalar
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

bool takla = false ;

//Heart Rate Sensor*********** Gerekli Tanımlamalar (BPM)
int sensorPin = 33;      
int threshold  = 2960 ;                      // A0 is the input pin for the heart rate sensor
float sensorValue = 0;                             // Variable to store the value coming from the sensor
int count = 0;
unsigned long starttime = 0;
int heartrate = 0;
boolean counted = false;
//***********************FireBase Kütüphanesi ve gerekli tanımlamalar
#include <Arduino.h>
#if defined(ESP32)
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>

//Veritabanına düzgün bağlanmak için gerekli olan ilaveler
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

// Ağ bilgilerinizi girin
#define WIFI_SSID "SUPERONLINE_Wi-Fi_4474"
#define WIFI_PASSWORD "anadolutat1071"

// Firebase proje API Key'i girin
#define API_KEY "AIzaSyBjJyNrBat2G4wrDVsOX3FW9CIYM2sdfP0"

// Veritabanı URL'sini girin */
#define DATABASE_URL "https://smart-motorcycle-helmet-836e7-default-rtdb.firebaseio.com/"

//Bir Firebase veri objesi oluşturalım
FirebaseData fbdo;
//yetki ve ayar nesneleri oluşturalım
FirebaseAuth auth;
FirebaseConfig config;
//***********************gerekli değişken tanımları (Bulut Ortamı için)
unsigned long sendDataPrevMillis = 0;
unsigned long getDataPrevMillis = 0;
bool signupOK = false;
int intValue=0;
bool wifi = false ;


void setup (void) //SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP
{
//**********i2c lcd ekran***************
lcd.begin();

pinMode(pinSarj100 , INPUT);
pinMode(pinSarj75 , INPUT);
pinMode(pinSarj50 , INPUT);
pinMode(pinSarj25 , INPUT);

//**********Kanama Sensörü**************
pinMode(rainPin, INPUT); 

//**************Kanama End*********************


Serial.begin (115200);           // Start Serial Communication @ 115200


//**********Gyroscope Sensor************
while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
  Serial.println("");
  delay(100);
//**********Gyroscope End**************
//**********Heart Rate Sensor***********

pinMode (18, OUTPUT);              

//******************End Heartrate***********************
//*************Wifi and Firebase Connection*************
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Ağa bağlanıyor");
  while (WiFi.status() != WL_CONNECTED) 
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Bağlandı. IP Adresi: ");
  wifi = true ;

  Serial.println(WiFi.localIP());
  Serial.println();

  /* yukarıdaki API keyi ayarlara atayalım */
  config.api_key = API_KEY;

  /* veritabanı URL'sini ayarlara atayalım */
  config.database_url = DATABASE_URL;

  /* giriş yapalım */
  if (Firebase.signUp(&config, &auth, "", "")) 
  {
    Serial.println("ok");
    signupOK = true;
  }
  else {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* token'in geçerlilik durumu kontrolü için gerekli */
  config.token_status_callback = tokenStatusCallback; 
//bağlantıyı başlatalım
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  
}

void loop () // LOOP  LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP
{
// *******************ı2c lcd ekran, şarj ve wifi kontrolü ************************
 lcd.setCursor(0,0); // İlk satırın başlangıç noktası
 if(wifi == true)
 {
  lcd.print("WiFi'a bagli.");
  delay(2000); 
 }
 else
 {
  lcd.print("Internet yok.");
  delay(2000); 
 }
 lcd.setCursor(0,1); 

 if (digitalRead(pinSarj100) == HIGH)
 {
  lcd.print("Sarj : %100");
  delay(2000);
 }
 else if(digitalRead(pinSarj75) == HIGH && digitalRead(pinSarj100) == LOW)
 {
  lcd.print("Sarj : %75");
  delay(2000); 
 }
 else if(digitalRead(pinSarj50) == HIGH && digitalRead(pinSarj75) == LOW)
 {
  lcd.print("Sarj : %50");
  delay(2000); 
 }
 else if(digitalRead(pinSarj25) == HIGH && digitalRead(pinSarj50) == LOW)
 {
  lcd.print("Sarj:%25 !!!");
  delay(2000); 
 }
 else
 {
  lcd.print("Sarj: ?"); 
  delay(2000); 
 }

 //lcd.clear();

//******************Kanama Sensor Loop*************************************
  int sensorValue = analogRead(rainPin);
  Serial.print(sensorValue);
  if(sensorValue < thresholdValue)
  {
    Serial.println(" - Sürücünün Kafa Bölgesinde Kanaması Var. ");
    kanama = true ;
  }
  else 
  {
    Serial.println(" - Kanama Yok ");
    kanama = false ;
  }
//******************Gyroscope/Temperature Sensor Loop**********************
/* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print("\n");

  if(a.acceleration.x < 0) 
  {

  takla = true ;
  Serial.print("Sürücü Takla Attı. \n");  

  }
  else
  {

   takla = false ; 
   Serial.print("Sürücü Konumu Normal. \n"); 

  }
  Serial.print("Kask içi sıcaklık: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  
//******************Heart Rate Sensor Loop*********************
starttime = millis();

 while (millis()<starttime+10000)                   // Reading pulse sensor for 10 seconds
 {
  sensorValue = analogRead(sensorPin);
  delay(50);
  if (sensorValue > threshold && counted == false)  // Threshold value is 550 (~ 2.7V)
  {

  count++;
  /*Serial.print ("count = ");
  Serial.println (count); */
  digitalWrite (18,HIGH);
  delay (50);
  digitalWrite (18, LOW);
  counted = true;
  }
 
  else if (sensorValue < threshold)
  {
  counted = false;
  digitalWrite (13, LOW);
  }
}

heartrate = (count*6);                               // Multiply the count by 6 to get beats per minute
Serial.println ();
Serial.print ("BPM = ");
Serial.println (heartrate);                        // Display BPM in the Serial Monitor
count = 0;

delay(500);

  if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 15000 || sendDataPrevMillis == 0)) 
  {
    sendDataPrevMillis = millis();
    
    //**************Ortam Sıcaklığı verisinin bulut ortama yazılması************************
    if (Firebase.RTDB.setInt(&fbdo, "A/KalpAtis/int", heartrate)) 
    {
      Serial.println("YAZMA TAMAM");
      Serial.println("DİZİN: " + fbdo.dataPath());
      Serial.println("VERİ TİPİ: " + fbdo.dataType());
    }
    else 
    {
      Serial.println("HATA");
      Serial.println("HATA SEBEBİ: " + fbdo.errorReason());
      wifi = false ;
    }
//**************Kaskın X konumu ve bool shit  verisinin bulut ortama yazılması************************
    if (Firebase.RTDB.setInt(&fbdo, "A/KaskinXkonumu/int", a.acceleration.x)) 
    {
      Serial.println("YAZMA TAMAM");
      Serial.println("DİZİN: " + fbdo.dataPath());
      Serial.println("VERİ TİPİ: " + fbdo.dataType());
    }
    else 
    {
      Serial.println("HATA");
      Serial.println("HATA SEBEBİ: " + fbdo.errorReason());
      wifi = false ;
    }
//************** Uyarı bool shit  verisinin bulut ortama yazılması************************
    if (Firebase.RTDB.setInt(&fbdo, "A/Uyari/bool", uyari)) 
    {
      Serial.println("YAZMA TAMAM");
      Serial.println("DİZİN: " + fbdo.dataPath());
      Serial.println("VERİ TİPİ: " + fbdo.dataType());
    }
    else 
    {
      Serial.println("HATA");
      Serial.println("HATA SEBEBİ: " + fbdo.errorReason());
      wifi = false ;
    }

    if (Firebase.RTDB.setInt(&fbdo, "A/KaskTaklaDurumu/bool", takla)) 
    {
      Serial.println("YAZMA TAMAM");
      Serial.println("DİZİN: " + fbdo.dataPath());
      Serial.println("VERİ TİPİ: " + fbdo.dataType());
    }
    else 
    {
      Serial.println("HATA");
      Serial.println("HATA SEBEBİ: " + fbdo.errorReason());
      wifi = false ;
    }
    //**************Kask İçi Sıcaklık verisinin bulut ortama yazılması************************
    if (Firebase.RTDB.setInt(&fbdo, "A/KaskIciSicaklik/float", temp.temperature)) 
    {
      Serial.println("YAZMA TAMAM");
      Serial.println("DİZİN: " + fbdo.dataPath());
      Serial.println("VERİ TİPİ: " + fbdo.dataType());
    }
    else 
    {
      Serial.println("HATA");
      Serial.println("HATA SEBEBİ: " + fbdo.errorReason());
      wifi = false ;
    }
    //**************Kanama verisinin bulut ortama yazılması************************
    if (Firebase.RTDB.setInt(&fbdo, "A/Kanama/bool", kanama)) 
    {
      Serial.println("YAZMA TAMAM");
      Serial.println("DİZİN: " + fbdo.dataPath());
      Serial.println("VERİ TİPİ: " + fbdo.dataType());
    }
    else 
    {
      Serial.println("HATA");
      Serial.println("HATA SEBEBİ: " + fbdo.errorReason());
      wifi = false ;
    }
  }

     
delay(1500);
}
