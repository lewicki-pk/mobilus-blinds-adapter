#include <WiFi.h>
#include <ArduinoMqttClient.h>


void buttonOnOff(int pin);

void buttonOnOff(int pin1, int pin2);

bool displayEquals10();

void markSynchronizedIfBlind10Reached();

void onMqttMessage(int messageSize);

void reconnectWifiIfNeeded();

void setup_pins();

bool currentBlindKnown();

void synchronize();

void switchToBlindByNum(int num);

bool handleBlindCommand(const char *message, const char *operationPrefix, void (*callback)());

bool handleGlobalCommand(const char *message, const char *operationPrefix, void (*callback)());

void sendOnlineMessage();

void delayOnButtonPress();

void reconnectMqttIfNeeded();

void reconnectIfNeeded();

// const int LED_LEFT_PIN = 3;
const int LED_RIGHT_PIN = 36;

const int BUTTON_LEFT_PIN = 16;
const int BUTTON_RIGHT_PIN = 17;

const int BUTTON_UP_PIN = 18; // was 12
const int BUTTON_DOWN_PIN = 19; // was 3; // was 11

const int BUTTON_MID_PIN = 21; // was 1; // was 18

// #if __has_include("secrets.h")

// #include "secrets.h"

// #endif

// #ifndef SSID
#define SSID "SSID"
#define PASSWORD  "PASSWORD"
#define HOSTNAME  "core-mosquitto"
#define MQTT_BROKER "192.168.68.117"
// const IPAddress = new IPAddress(192,168,68,117);
const int MQTT_PORT = 1883;
const char *COMMAND_TOPIC = "mobilus/blinds/command";
const char *ONLINE_TOPIC = "mobilus/blinds/available";

const int DELAY_AFTER_COMMAND_MS = 1500;
const int DELAY_REMOTE_INACTIVE_MS = 1000;

const int DURATION_WAKING_PRESS_MS = 175; // was = 75;
const int DURATION_NORMAL_PRESS_MS = 135; // was = 35;
const int DELAY_BETWEEN_NAV_MS = 35;

const int DURATION_DISPLAY_DIGIT_PROBE_MS = 10;

const int DELAY_ONLINE_MESSAGE_MS = 1000;
const int DELAY_RECONNECTION_RETRIAL = 10000;

const int SEGMENT_INACTIVE_MVOLTS_THRESHOLD = 2000;

// position
int currentBlind = -1;

// last <action> timestamps
int lastOnlineMessageSentMillis = -DELAY_RECONNECTION_RETRIAL;

int lastCheckedConnectionMillis = -DELAY_RECONNECTION_RETRIAL;

int lastButtonPressMillis = -DELAY_RECONNECTION_RETRIAL;

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);


void setup() {
  setup_pins();
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setUsernamePassword("piotrMqtt", "trudneHaslo");

  // PIOTR START
  Serial.begin(19200);  
  Serial.println("--- Start Serial Monitor ---");
  // Serial.println(" Type in Box above, . ");
  // Serial.println("(Decimal)(Hex)(Character)");  
  Serial.println(); 
  // PIOTR END
}

void setup_pins() {
  pinMode(LED_RIGHT_PIN, INPUT);
//   pinMode(LED_LEFT_PIN, INPUT);

  pinMode(BUTTON_LEFT_PIN, OUTPUT);
  digitalWrite(BUTTON_LEFT_PIN, 0);
  pinMode(BUTTON_RIGHT_PIN, OUTPUT);
  digitalWrite(BUTTON_RIGHT_PIN, 0);

  pinMode(BUTTON_UP_PIN, OUTPUT);
  digitalWrite(BUTTON_UP_PIN, 0);
  pinMode(BUTTON_DOWN_PIN, OUTPUT);
  digitalWrite(BUTTON_DOWN_PIN, 0);

  pinMode(BUTTON_MID_PIN, OUTPUT);
  digitalWrite(BUTTON_MID_PIN, 0);
}

void loop() {
  // Serial.print("Current blind =" + String(currentBlind));
  if (!currentBlindKnown()) {
    Serial.println(" - running synchronize...");
    synchronize();
    Serial.println(" - synchronize finished...");
    return;
  }
  // Serial.println(" - now in an endless loop...");

  // unsigned long startTimeT = millis();
  // while (millis() - startTimeT < 5000) {
  //   if (Serial.available()) {
  // //     // Read the input (you can modify this part to handle the input as needed)
  //     char inputT = Serial.read();
  //     // input = Serial.read();
  //     Serial.print("Received input: ");
  //     Serial.println(inputT);
  //     switch (inputT) {
  //         case 'k': {
  //             // Read from analog input 0
  //             int nowT = millis();
  //             int nowPlusDurationT = nowT + DURATION_DISPLAY_DIGIT_PROBE_MS;
  //             while (millis() < nowPlusDurationT) {
  //                 int milliVoltsT = analogReadMilliVolts(LED_RIGHT_PIN);
  //                 Serial.println("Milivolts = " + String(milliVoltsT));
  //             }
  //             break;
  //         }

  //         case 'w':
  //             buttonOnOff(BUTTON_UP_PIN);
  //             delay(500);
  //             break;

  //         case 's':
  //             buttonOnOff(BUTTON_DOWN_PIN);
  //             delay(500);
  //             break;

  //         case 'a':
  //             buttonOnOff(BUTTON_LEFT_PIN);
  //             delay(500);
  //             break;

  //         case 'd':
  //             buttonOnOff(BUTTON_RIGHT_PIN);
  //             delay(500);
  //             break;

  //         case 'e':
  //             buttonOnOff(BUTTON_MID_PIN);
  //             delay(500);
  //             break;

  //         default:          
  //             // Handle other cases if needed
  //             break;
  //     }
  //     // Exit the loop
  //     break;
  //   }
  // }

  // Serial.println("Before delay(DELAY_BETWEEN_NAV_MS) - line 189");
  delay(DELAY_BETWEEN_NAV_MS);
  reconnectIfNeeded();

  if (mqttClient.connected()) {
    Serial.println("Mqtt client connected!");
    sendOnlineMessage();
    mqttClient.poll();
  } else {
    Serial.println("Mqtt client not connected and able to send");
  }
}

void synchronize() {
  // buttonOnOff(BUTTON_LEFT_PIN); // FIXME: to dziala ale tylko raz
  // delay(DELAY_BETWEEN_NAV_MS);
  buttonOnOff(BUTTON_LEFT_PIN);
  delay(500);
  markSynchronizedIfBlind10Reached();
}

bool currentBlindKnown() { return currentBlind != -1; }

void markSynchronizedIfBlind10Reached() {
  if (displayEquals10()) {
    currentBlind = 10;
  }
}

bool displayEquals10() {
  int now = millis();
  int nowPlusDuration = now + DURATION_DISPLAY_DIGIT_PROBE_MS;
  int iteration = 1;
  while (millis() < nowPlusDuration) {
    Serial.println("Inside while loop. Iteration=" + String(iteration++));
    int milliVolts = analogReadMilliVolts(LED_RIGHT_PIN);
    Serial.print("Milivolts value=" + String(milliVolts));
    Serial.println(" should be bigger than threshol=" + String(SEGMENT_INACTIVE_MVOLTS_THRESHOLD));
    // Serial.println("There is something wrong with output from millis(), now and some other duration");
    Serial.print("millis()=" + String(millis()));
    Serial.print(" is smaller than now+DURATION_DISPLAY_DIGIT_PROBE_MS=" + String(nowPlusDuration));
    Serial.println(" then finish the loop.");
    if (milliVolts > SEGMENT_INACTIVE_MVOLTS_THRESHOLD) { // tu wchodzimy zawsze od razu.
      return true;
    }
  }
  return false;
}

void reconnectIfNeeded() {
  int now = millis();
  if (now - lastCheckedConnectionMillis > DELAY_RECONNECTION_RETRIAL) {
    Serial.println("Inside reconnectIfNeeded() - true-> reconnect is needed");
    while (WiFi.status() != WL_CONNECTED) {
      reconnectWifiIfNeeded();
    }
    while (!mqttClient.connected()) {
      reconnectMqttIfNeeded();
    }

    lastCheckedConnectionMillis = now;
  }
}

void reconnectWifiIfNeeded() {
  // Serial.println("Inside reconnectWifiIfNeeded()");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Wifi connected");
    return;
  } else {
    Serial.println("Wifi not connected- reconnecting");
  }

  WiFi.begin(SSID, PASSWORD);
  for (int i = 0; i < 10; i++) {
    delay(1000);
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Wifi connected!");
      // break;
      return;
    }
  }
}

void reconnectMqttIfNeeded() {
  Serial.println("Inside reconnectMqttIfNeeded() - connecting mqtt");
  // bool reconnected = mqttClient.connect(MQTT_BROKER, MQTT_PORT);
  bool reconnected = mqttClient.connect(IPAddress(192,168,68,117), MQTT_PORT);
  if (reconnected) {
    Serial.print("Subscribing to a topic: ");
    Serial.println(COMMAND_TOPIC);
    mqttClient.subscribe(COMMAND_TOPIC);
  } else {
    Serial.println("Failed to reconnect to mqtt");
  }
}

void sendOnlineMessage() {
  Serial.println("Inside sendOnlineMessage()");
  int now = millis();
  if (now - lastOnlineMessageSentMillis > DELAY_ONLINE_MESSAGE_MS) {
    mqttClient.beginMessage(ONLINE_TOPIC);
    mqttClient.print("online");
    mqttClient.endMessage();
    lastOnlineMessageSentMillis = now;
  }
}

void moveUp() {
  buttonOnOff(BUTTON_UP_PIN);
  delay(DELAY_AFTER_COMMAND_MS);
}

void moveDown() {
  buttonOnOff(BUTTON_DOWN_PIN);
  delay(DELAY_AFTER_COMMAND_MS);
}

void stop() {
  buttonOnOff(BUTTON_MID_PIN);
  delay(DELAY_AFTER_COMMAND_MS);
}

void nop() {

}

void resynchronize() {
  currentBlind = -1;
}

void enterLeaveBlindProgramming() {
  buttonOnOff(BUTTON_MID_PIN, BUTTON_UP_PIN);
  delay(DELAY_AFTER_COMMAND_MS);
}

void onMqttMessage(int messageSize) {
  while (mqttClient.available()) {
    char message[100];
    mqttClient.read((uint8_t *) message, 100);

    handleBlindCommand(message, "UP ", moveUp) ||
    handleBlindCommand(message, "DO ", moveDown) ||
    handleBlindCommand(message, "ST ", stop) ||
    handleBlindCommand(message, "PR ", enterLeaveBlindProgramming) ||
    handleBlindCommand(message, "NO ", nop) ||
    handleGlobalCommand(message, "SY", resynchronize);
  }
}

bool handleBlindCommand(const char *message, const char *operationPrefix, void (*callback)()) {
  if (strncasecmp(message, operationPrefix, 3) == 0) {
    int num = atoi(message + 3);
    if (num != 0) {
      switchToBlindByNum(num);
      delay(DELAY_BETWEEN_NAV_MS);
      callback();
    }
    return true;
  }
  return false;
}

bool handleGlobalCommand(const char *message, const char *operationPrefix, void (*callback)()) {
  if (strncasecmp(message, operationPrefix, strlen(operationPrefix)) == 0) {
      callback();
      return true;
  }
  return false;
}

void buttonOnOff(int pin) {
  digitalWrite(pin, 1);
  delayOnButtonPress();
  digitalWrite(pin, 0);
}

void delayOnButtonPress() {
  int now = millis();
  if (now - lastButtonPressMillis > DELAY_REMOTE_INACTIVE_MS) {
    delay(DURATION_WAKING_PRESS_MS);
  } else {
    delay(DURATION_NORMAL_PRESS_MS);
  }
  lastButtonPressMillis = now;
}

void buttonOnOff(int pin1, int pin2) {
  digitalWrite(pin1, 1);
  digitalWrite(pin2, 1);
  delayOnButtonPress();
  digitalWrite(pin1, 0);
  digitalWrite(pin2, 0);
}

void switchToBlindByNum(int num) {
  int delta = num - currentBlind;
  if (delta != 0 && abs(delta) < 99) {
    for (int i = 0; i < abs(delta); i++) {
      if (i > 0) {
        delay(DELAY_BETWEEN_NAV_MS);
      }
      buttonOnOff(delta > 0 ? BUTTON_RIGHT_PIN : BUTTON_LEFT_PIN);
      currentBlind = num;
    }
  }
}
