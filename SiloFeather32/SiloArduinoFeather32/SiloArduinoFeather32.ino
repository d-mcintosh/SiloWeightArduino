#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <HX711_ADC.h>          // https://github.com/olkal/HX711_ADC

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
float txValue = 0;
int buttonPin = 15;
int buttonState = 0; 

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b" // UART service UUID
#define CHARACTERISTIC_UUID_RX "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_TX "4fafc201-1fb5-459e-8fcc-c5c9c331914b"

HX711_ADC LoadCell(17, 16);

// timer for reading load cell
long t = 0;
long p = 0;

void setupADC() {
  Serial.println("Setting up Scale");
  LoadCell.begin();
  long stabilisingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilising time
  LoadCell.start(stabilisingtime);
  LoadCell.setCalFactor(436.0); // user set calibration factor (float)
  Serial.println("Startup and tare is complete");
}
 
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true; 
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");

        for (int i = 0; i < rxValue.length(); i++) {
          Serial.print(rxValue[i]);
        }

        Serial.println();

        // Do stuff based on the command received from the app
        if (rxValue.find("A") != -1) { 
          Serial.print("Turning ON!");
        //  digitalWrite(LED, HIGH);
        }
        else if (rxValue.find("B") != -1) {
          Serial.print("Turning OFF!");
         // digitalWrite(LED, LOW);
        }

        Serial.println();
        Serial.println("*********");
      }
    }
};

/*
void BLEServer::disconnectClient() {
   ESP_LOGD(LOG_TAG, ">> disconnectClient()");
   esp_err_t errRc = ::esp_ble_gatts_close(getGattsIf(), getConnId());
   if (errRc != ESP_OK) {
      ESP_LOGE(LOG_TAG, "esp_ble_gatts_close: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
      return;
   }
   ESP_LOGD(LOG_TAG, "<< disconnectClient()");
} // disconnectClient

*/

void redLED() {
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(12, HIGH);    // turn the LED off by making the voltage LOW
  digitalWrite(27, HIGH);    // turn the LED off by making the voltage LOW  
  Serial.println("red");
}

void blueLED() {
  digitalWrite(13, HIGH);    // turn the LED off by making the voltage LOW
  digitalWrite(12, HIGH);    // turn the LED off by making the voltage LOW
  digitalWrite(27, LOW);    // turn the LED off by making the voltage LOW  
  Serial.println("blue");
}

void greenLED() {
  digitalWrite(13, HIGH);    // turn the LED off by making the voltage LOW
  digitalWrite(12, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(27, HIGH);    // turn the LED off by making the voltage LOW  
  Serial.println("green");
}

void setup() {
  Serial.begin(115200);

  pinMode(buttonPin, INPUT_PULLUP);
   //  attachInterrupt(digitalPinToInterrupt(buttonPin), BLEServer::disconnectClient(), FALLING);

   // Set LED out
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(23, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(1, OUTPUT);


  redLED();


  // Create the BLE Device
  BLEDevice::init("TEST Silo"); // Give it a name

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
                      
  pCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  Serial.println("wait for advertising");
  pServer->getAdvertising()->addServiceUUID(SERVICE_UUID);  // <- added this
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
  
  blueLED();
 
  setupADC();

  greenLED();
}

/**
 * Get the load cell value
 */
float getLoadValue() {

  LoadCell.update();

  float i = LoadCell.getData();

  return i;
}

/**
 * Tare (calibrate) the load cell
; */
void performTare() {
  LoadCell.tareNoDelay();

  //check if last tare operation is complete
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }  
}

void loop() {
  if (deviceConnected) {

   

    float txValue = getLoadValue() * 1;
    char txString[8];
    dtostrf(txValue, 1, 2, txString);
    //Serial.println(txString , 1);
      
    pCharacteristic->setValue(txString);
      // pCharacteristic->setValue("Hello!");
      pCharacteristic->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txValue);
      Serial.println(" ***");
      } 

    delay(1000);
}
