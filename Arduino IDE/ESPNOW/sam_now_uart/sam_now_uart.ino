#include <Arduino.h>
#include <ESP8266SAM_ES.h>
#include <AudioOutputI2S.h>
#include <esp_now.h>
#include <WiFi.h>

//pins settings for input for SR model
#define button1   25   //red 
#define button2   26   //blue 
#define button3   27   //green led

// // Define TX and RX pins for UART (change if needed)
// #define TXD1 17
// #define RXD1 16

#define LED_P     2    //on board red led
AudioOutputI2S *out = NULL;

// // Use Serial1 for UART communication
// HardwareSerial mySerial(2);
// char linea[60];
// int pos = 0;
// int punt_linea = 0;
// int llm;          // FLAG TO READ  LLM output

// Flag input to store values
int in_no;
int in_roll;
int in_throw;

// Set internal flag to store info based on SR input for action done
int sr_no;
int sr_roll;
int sr_throw;

char story[64];

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x40,0x22,0xD8,0xEA,0x0C,0xD4};       //ESP32 MAC Address

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int flag;
  int nof;    // no action flag
  int rollf;  // roll action flag
  int throwf; // throw action flag
  char message[64];
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Create a struct_message to hold incoming flags
struct_message inData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&inData, incomingData, sizeof(inData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  in_no = inData.nof;
  in_roll = inData.rollf;
  in_throw = inData.throwf;
  strcpy(story, inData.message);
  Serial.print("Not: ");
  Serial.printf("%d \n", in_no);
  Serial.print("Roll: ");
  Serial.printf("%d \n", in_roll);
  Serial.print("Throw: ");
  Serial.printf("%d \n", in_throw);
  Serial.println();
  Serial.print("Char:");
  Serial.println(story);
}

void setup()
{
  Serial.begin(115200);
  // mySerial.begin(9600, SERIAL_8N1, RXD1, TXD1);  // UART setup
  // Serial.println("ESP32 UART Receiver");

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

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

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
  // llm=0;
  ESP8266SAM_ES *sam = new ESP8266SAM_ES;
  //sam -> SetVoice (sam -> SAMVoice::VOICE_SAM);
  sam -> SetRegion (sam -> SAMRegion::REGION_OT);
  //sam -> SetPhonetic (true);
  sam -> SetPitch (91);
  sam -> SetMouth (130);
  sam -> SetThroat (110);
  sam -> SetSpeed (90);

  // if (mySerial.available()) {
    // // Read data and display it
    // String message = mySerial.readStringUntil('\n');
    // Serial.println("Received: " + message);

    // // Clear the previous line buffer
    // memset(linea, 0, sizeof(linea)); // Clear the linea array
    // pos = 0; // Reset position
    // punt_linea = 0; // Reset punt_linea

  //   // Copy the message into the linea array
  //   for (char cr : message) {
  //       if (pos < 59) {
  //           if (cr == '\n') {
  //               break; // Stop processing if a newline is encountered
  //           }
  //           linea[pos++] = cr; // Add character to linea
  //       }
  //   }

  //   // Null-terminate the string
  //   linea[pos] = 0;
  // }
  //   if (llm==1){
  //   sam->Say(out, linea);
  //   delay(500);
  //   delete sam;
  //   llm=0;
  //   }

  

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
    sr_throw = 1;
    sr_roll = 0;
    sr_no = 0;
    digitalWrite(LED_P,LOW);
  }
  else if(digitalRead(button1) == 0 && digitalRead(button2) == 1 && digitalRead(button3) == 1)       //The ball was rolled
  {
    digitalWrite(LED_P, HIGH);
    sr_throw = 0;
    sr_roll = 1;
    sr_no = 0;
    digitalWrite(LED_P,LOW);
  }
  else if(digitalRead(button1) == 1 && digitalRead(button2) == 1 && digitalRead(button3) == 1)       //The ball did not move
  {
    digitalWrite(LED_P, HIGH);
    sr_throw = 0;
    sr_roll = 0;
    sr_no = 1;
    digitalWrite(LED_P,LOW);
  }
  else
  {
    myData.flag=0;
    sr_throw = 0;
    sr_roll = 0;
    sr_no = 0;
  }
  
  if (sr_throw == in_throw && sr_throw == 1)
  {
    sam->Say(out, "Congratulations !");
    delay(200);
    // llm=1;

  }
  else if (sr_roll == in_roll && sr_roll == 1)
  {
    sam->Say(out, "Congratulations !");
    delay(500);
    // llm=1;
  }
  else if (sr_no == in_no && sr_no == 1)
  {
    sam->Say(out, "Congratulations !");
    delay(500);
    // llm=1;
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
    //Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(1000);
}