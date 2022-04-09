#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Servo.h>

Servo yaw;  // create servo object to control a servo
Servo pitch;  // create servo object to control a servo
int k=0;
// Structure example to receive data
// Must match the sender structure
typedef struct test_struct {
    int x;
    int y;
} test_struct;

// Create a struct_message called myData
test_struct myData;

// Callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&myData, incomingData, sizeof(myData));
//  Serial.print("Bytes received: ");
//  Serial.println(len);
  Serial.print("\n\nx: ");
 k=0;
 Serial.println(k);
 Serial.println(myData.x);
  Serial.print("Z: ");
  Serial.println(myData.y);

  Serial.print("\n\n ");
//  yaw.write(90+myData.y);
//  pitch.write(90+myData.x);
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  yaw.attach(14); 
  pitch.attach(12); 
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // Serial.println("hui");
  k++;
  if(k>0)
  {  yaw.write(90+1.5*myData.y);
 pitch.write(90+1.5*myData.x);}
Serial.println(k);}
