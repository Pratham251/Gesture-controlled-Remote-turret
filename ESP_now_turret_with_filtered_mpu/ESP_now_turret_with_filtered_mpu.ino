#include <ESP8266WiFi.h>
#include <espnow.h>
#include<Wire.h>


unsigned long timer = 0;


int16_t gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, temperature, acc_total_vector;
float angle_pitch, angle_roll, angle_yaw;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;
float elapsedTime;
long Time, timePrev, time2;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;


 
double x;
double y;
double z;

uint8_t broadcastAddress1[] = {0xE8, 0xDB, 0x84, 0xDD, 0x42, 0x39};
uint8_t broadcastAddress2[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Structure example to send data
// Must match the receiver structure
typedef struct test_struct {
    int X;
    int Z;
} test_struct;

test_struct test;



unsigned long lastTime = 0;  
unsigned long timerDelay = 5;  // send readings timer


void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  char macStr[18];
  Serial.print("Packet to:");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
         mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");
  }
  else{
    Serial.println("Delivery fail");
  }
}


void setup() {
  // Init Serial Monitor


  
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  delay(1000);
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++) {
    if (cal_int % 125 == 0)Serial.print(".");
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 14);
    while (Wire.available() < 14);
    acc_x = Wire.read() << 8 | Wire.read();
    acc_y = Wire.read() << 8 | Wire.read();
    acc_z = Wire.read() << 8 | Wire.read();
    temperature = Wire.read() << 8 | Wire.read();
    gyro_x = Wire.read() << 8 | Wire.read();
    gyro_y = Wire.read() << 8 | Wire.read();
    gyro_z = Wire.read() << 8 | Wire.read();
    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;
    GPOS = (1 << 0); GPOS = (1 << 2); GPOS = (1 << 12); GPOS = (1 << 14);
    delayMicroseconds(1000);
    GPOC = (1 << 0); GPOC = (1 << 2); GPOC = (1 << 12); GPOC = (1 << 14);
    delayMicroseconds(1000);
    yield();///////////////////////////////////////////////////////////////////
  }
  gyro_x_cal /= 2000;
  gyro_y_cal /= 2000;
  gyro_z_cal /= 2000;

  Time = micros();
  

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

   // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_add_peer(broadcastAddress1, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
}


void loop() {


timePrev = Time;
  Time = micros();
  elapsedTime = (float)(Time - timePrev) / (float)1000000;
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);
  while (Wire.available() < 14);
  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();
  temperature = Wire.read() << 8 | Wire.read();
  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;
  angle_pitch += gyro_x * elapsedTime * 0.01526717557;
  angle_roll += gyro_y * elapsedTime * 0.01526717557;
  angle_yaw += gyro_z * elapsedTime * 0.01526717557;
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));
  angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;
  angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;
  angle_pitch_acc += 0;
  angle_roll_acc += 0;
  if (set_gyro_angles) {
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
  }
  else {
    angle_pitch = angle_pitch_acc;
    angle_roll = angle_roll_acc;
    set_gyro_angles = true;
  }
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;
//Serial.println(angle_yaw);
Serial.println( angle_roll_output);


y=angle_roll_output;
z=angle_yaw;


 test.X = -(y);
 test.Z =(z);

 esp_now_send(broadcastAddress1, (uint8_t *) &test, sizeof(test));
}






float restrict(float hui)

{if(hui>180)
hui=hui-360;


return hui;  
  }
