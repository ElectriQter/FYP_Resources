#include <Arduino.h>
#include <ESP8266SAM_ES.h>
#include <AudioOutputI2S.h>
#include <esp_now.h>
#include <WiFi.h>

//pins settings for input for SR model
#define button1   25   //red 
#define button2   26   //blue 
#define button3   27   //green led

#define LED_P     2    //on board red led
AudioOutputI2S *out = NULL;

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x40,0x22,0xD8,0xEA,0x0C,0xD4};       //ESP32 MAC Address

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int flag;
} struct_message;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


void setup()
{
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  //Set up the SAM TTS and Amp pins
  out = new AudioOutputI2S();
  out->begin();
  out -> SetGain(0.2);
  out -> SetPinout(12,13,14);
  //bool SetPinout(int bclkPin, int wclkPin, int doutPin);

  //pin setup for button input from SR
  pinMode(button1, INPUT_PULLDOWN);
  pinMode(button2, INPUT_PULLDOWN);
  pinMode(button3, INPUT_PULLDOWN);

  //on board led to know when output from the SAM TTS
  pinMode(LED_P, OUTPUT);
  digitalWrite(LED_P,LOW);
}

void loop()
{

  ESP8266SAM_ES *sam = new ESP8266SAM_ES;
  //sam -> SetVoice (sam -> SAMVoice::VOICE_SAM);
  sam -> SetRegion (sam -> SAMRegion::REGION_OT);
  //sam -> SetPhonetic (true);
  sam -> SetPitch (91);
  sam -> SetMouth (130);
  sam -> SetThroat (110);
  sam -> SetSpeed (90);

  if (digitalRead(button1) == 1 && digitalRead(button2) == 0 && digitalRead(button3) == 0)          //Throw the ball
  {
    digitalWrite(LED_P, HIGH);
    sam->Say(out, "Ttherow the ball !");
    delay(200);
    digitalWrite(LED_P,LOW);
    myData.flag= 1;
  }
  else if (digitalRead(button1) == 0 && digitalRead(button2) == 1 && digitalRead(button3) == 0)     //Roll the ball
  {
    digitalWrite(LED_P, HIGH);
    sam->Say(out, "Rolle the ball !");
    delay(200);
    digitalWrite(LED_P,LOW);
    myData.flag= 1;
  }
  else if(digitalRead(button1) == 0 && digitalRead(button2) == 0 && digitalRead(button3) == 1)      //Hold the ball
  {
    digitalWrite(LED_P, HIGH);
    sam->Say(out, "Hold the ball !");
    delay(200);
    digitalWrite(LED_P,LOW);
    myData.flag= 1;
  }
  else if(digitalRead(button1) == 1 && digitalRead(button2) == 0 && digitalRead(button3) == 1)       //The ball was thrown
  {
    digitalWrite(LED_P, HIGH);
    sam->Say(out, "Congratulations !");
    delay(200);
    sam->Say(out, "The short story is thrown");    // to add with LLM output after
    digitalWrite(LED_P,LOW);
  }
  else if(digitalRead(button1) == 0 && digitalRead(button2) == 1 && digitalRead(button3) == 1)       //The ball was rolled
  {
    digitalWrite(LED_P, HIGH);
    sam->Say(out, "Congratulations !");
    delay(200);
    sam->Say(out, "The short story is rolled");    // to add with LLM output after
    digitalWrite(LED_P,LOW);
  }
  else if(digitalRead(button1) == 1 && digitalRead(button2) == 1 && digitalRead(button3) == 1)       //The ball did not move
  {
    digitalWrite(LED_P, HIGH);
    sam->Say(out, "Congratulations !");
    delay(200);
    sam->Say(out, "The short story is no action");    // to add with LLM output after
    digitalWrite(LED_P,LOW);
  }
  else
  {
    myData.flag=0;
  }
/* 
  // This section used for testing model on its own 
  sam->Say(out, "He like to play with his friends");
  delay(1000);
  sam->Say(out, "Hey My name is Tom");
  delay(1000);*/
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  delete sam;
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(1000);
}
