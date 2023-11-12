#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>

// MPU-6050 dependancies
#include "I2Cdev.h"
#include <MPU6050.h>
#include <Wire.h>

#include "config.h"


// put function declarations here:
void readDataFromWiFiBuffer();
void handleDisconnection();
void sendPingGetRequest();
bool isServerConnectionPossible();
void printWifiStatus();
void initWiFi();
void initMPU6050();
int calibrateSensors(int calibrationsDonePrevious, int additionalCalibrations);
void updateSmoothed();
void printSmoothed();
void postMPU6050Data();


//---------------------------------------------------------------
//  Constants
//---------------------------------------------------------------
const int iAx = 0;  // accelerometer x index
const int iAy = 1;  // accelerometer y index
const int iAz = 2;  // accelerometer x index
const int iGx = 3;  // gyroscope x index
const int iGy = 4;  // gyroscope y index
const int iGz = 5;  // gyroscope z index

const unsigned long reconnectionTimeInterval = 30000; // 300 seconds between each reconnection check


unsigned long previousMillis = 0;

int status = WL_IDLE_STATUS;

int smoothed[6];    // smoothed accelerometer and gyroscope data 

WiFiClient client; 
MPU6050 accelgyro;


void setup() {
  Serial.begin(9600);

  while (!Serial) {;}

  initMPU6050();
  // initWiFi();

  // while(!isServerConnectionPossible()) {;}

  // sendPingGetRequest();
}

void loop() {

  updateSmoothed(); 
  printSmoothed();

}

/**
 * @brief Connects the ESP device to wifi
 * 
 */
void initWiFi() {
  // attempt to connect to the wifi
  WiFi.mode(WIFI_STA); // set device to station mode
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(NETWORK_SSID);
    WiFi.begin(NETWORK_SSID, NETWORK_PASSWORD);

    status = WiFi.waitForConnectResult();

    Serial.print("Current WiFi status: ");
    Serial.println(status);

    if (status != WL_CONNECTED) delay(10000); // wait 10 seconds for connection:
  }

  // print current WiFi status
  Serial.println("Connected to wifi");
  printWifiStatus();
}

void initMPU6050() {
  Wire.begin();

  Serial.println("Initializing IC2 devices");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  if (!accelgyro.testConnection()) {
    Serial.println("MPU6050 connection failed. Please check your connections.");
    while (1);
  }
  Serial.println("Connection Successful");

  // calibrate sensors 
  int calibrationCount = 0;
  calibrationCount = calibrationCount + calibrateSensors(calibrationCount, 6);
  // calibrationCount = calibrationCount + calibrateSensors(calibrationCount, 1);
  // calibrationCount = calibrationCount + calibrateSensors(calibrationCount, 1);
}

int calibrateSensors(int calibrationsDonePrevious, int additionalCalibrations) {
  for(int i = 0; i < additionalCalibrations; i++) {
    accelgyro.CalibrateAccel(i);
    accelgyro.CalibrateGyro(i);
  }
  Serial.printf("%d Readings Taken\n", (calibrationsDonePrevious + additionalCalibrations) * 100);
  accelgyro.PrintActiveOffsets();
  Serial.println();
  return calibrationsDonePrevious + additionalCalibrations;
}

void updateSmoothed(){ 
  int16_t RawValue[6];
  int i;
  long Sums[6];
  int N = 1000; 

  for (i = iAx; i <= iGz; i++) { 
    Sums[i] = 0; 
  }
    //   unsigned long Start = micros();
    

  for (i = 1; i <= N; i++)
    { // get sums
      accelgyro.getMotion6(&RawValue[iAx], &RawValue[iAy], &RawValue[iAz], 
                            &RawValue[iGx], &RawValue[iGy], &RawValue[iGz]);
      // if ((i % 500) == 0) Serial.print('.');

      // delayMicroseconds(3150); // hold sampling to 200hz
      delayMicroseconds(100);
      for (int j = iAx; j <= iGz; j++) {
        Sums[j] = Sums[j] + RawValue[j];
      }
    } // get sums
//    unsigned long usForN = micros() - Start;
//    Serial.print(" reading at ");
//    Serial.print(1000000/((usForN+N/2)/N));
//    Serial.println(" Hz");
  for (i = iAx; i <= iGz; i++){ 
    // smoothed[i] = (Sums[i] + N/2) / N ; 
    smoothed[i] = RawValue[i];
  }
}


void printSmoothed() {
  Serial.printf(
    "Acceleration: X: %d Y: %d Z: %d (m/s^2)\n",
    smoothed[iAx], smoothed[iAy], smoothed[iAz]
  );
  Serial.printf(
    "Orientation: X: %d Y: %d Z: %d (rad/s)\n",
    smoothed[iGx], smoothed[iGy], smoothed[iGz]
  );
  Serial.println("");
}



/**
 * @brief Reads any data what was sent to the device over WiFi.
 * 
 */
void readDataFromWiFiBuffer() {
  while (client.available()) {
    char c = client.read();
    Serial.write(c);
  }
}

/**
 * @brief If the server becomes unavailible, hang the client.
 * 
 */
void handleDisconnection() {
  if (!client.connected()) {
    Serial.println();
    Serial.println("disconnecting from server.");
    client.stop();
    while (true);
  }
}

/**
 * @brief A test which checks whether the server is able to take requests as expected
 * 
 * @return true 
 * @return false 
 */
bool isServerConnectionPossible() {
  if (client.connect(SERVER_DEST, SERVER_PORT)) {
    Serial.println("Connected to Server");
    client.stop();
    Serial.println("\nDisconnected from server");
    return true;
  } else {
    Serial.println("Ping Attempted: client couldn't connect");
  }
  return false;
}

/**
 * @brief 
 * @return is Recieved Properly? 
 */
void sendPingGetRequest() {
  if (client.connect(SERVER_DEST, SERVER_PORT)) {
    Serial.println("Connected to Server");

    // POST request
    client.println("GET /ping HTTP/1.1");
    // client.println("Host: " + String(SERVER_URL) + ":" + String(SERVER_PORT));
    // client.println("Connection: close");
    client.println();

    // wait for the response 
    while (client.connected() && !client.available()) {
      delay(10);
    }

    // Print the response
    while (client.available()) {
      char c = client.read();
      Serial.print(c);
    }

    client.stop();
    Serial.println("\nDisconnected from server");
  } else {
    Serial.println("Ping Attempted: client couldn't connect");
  }
}

// void postMPU6050Data(mpuMotion6 ) {
//   if (client.connect(SERVER_DEST, SERVER_PORT)) {
//     Serial.println("Connected to Server");

//     // POST request
//     client.println("GET /ping HTTP/1.1");
//     // client.println("Host: " + String(SERVER_URL) + ":" + String(SERVER_PORT));
//     // client.println("Connection: close");
//     client.println();

//     // wait for the response 
//     while (client.connected() && !client.available()) {
//       delay(10);
//     }

//     // Print the response
//     while (client.available()) {
//       char c = client.read();
//       Serial.print(c);
//     }

//     client.stop();
//     Serial.println("\nDisconnected from server");
//   } else {
//     Serial.println("Ping Attempted: client couldn't connect");
//   }
// }

/**
 * @brief A helper print function which should display in the serial monitor the current status of the WiFi connection.
 * 
 */
void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}