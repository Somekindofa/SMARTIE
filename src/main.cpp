#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>

const char* ssid = "Livebox-6A10";
const char* password = "xf3wgEcRLu6Gf7bLkV";
const char* mqtt_server = "192.168.1.14";
const char* ledTopic = "home/esp/led";

const int ledPin = LED_BUILTIN;                        // Replace with the actual GPIO pin connected to the LED
const int buttonPin = D1;                     // Replace with the actual GPIO pin connected to SW
volatile bool ledState = false;               // This will keep track of the LED state
volatile unsigned long lastDebounceTime = 0;  // Timestamp to track the last interrupt event
unsigned long debounceDelay = 50;             // Debounce delay in milliseconds

// Forward declaration of the callback function
void IRAM_ATTR handleButtonPress();

WiFiClient espClient;
PubSubClient client(espClient);


void IRAM_ATTR handleButtonPress() {
  // Check if we are stable from the last debounce period
  unsigned long currentTime = millis();
  if ((currentTime - lastDebounceTime) > debounceDelay) {
    // Toggle the LED state
    ledState = !ledState;
    lastDebounceTime = currentTime;
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0'; // Null-terminate the payload byte array
  String message = String((char*)payload);

  if (String(topic) == ledTopic) {
    if (message.equals("ON")) {
      digitalWrite(ledPin, LOW); // Turn the LED on (Note: LOW is on for most ESP8266 built-in LEDs)
    } else if (message.equals("OFF")) {
      digitalWrite(ledPin, HIGH); // Turn the LED off
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), "homeassistant", "ohhooNahjiphiewaagahb8thoh9phaihah9Ootohchoh5oquahghaivob6xoh1ja")) {
      Serial.println("connected");

      client.publish("home/esp/mac", WiFi.macAddress().c_str());
      client.publish("home/esp/ip", WiFi.localIP().toString().c_str());
      Serial.println("published IP and MAC");

      client.subscribe(ledTopic);

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup_wifi() {
  delay(10);
  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
}

void setup() {
  pinMode(ledPin, OUTPUT);     // Initialize the LED pin as an output
  pinMode(buttonPin, INPUT_PULLUP); // Initialize the button pin as an input with internal pull-up resistor
  digitalWrite(ledPin, HIGH);  // Turn the LED off
  // Attach the interrupt to the button pin
  attachInterrupt(digitalPinToInterrupt(buttonPin), handleButtonPress, RISING); // Call the handleButtonPress function when the button is pressed. FALLING means the interrupt will be triggered when the pin goes from HIGH to LOW
  Serial.begin(115200);

  setup_wifi(); // Connect to WiFi

  client.setServer(mqtt_server, 1883); // Set the MQTT broker details on the client
  client.setCallback(callback); // Set the callback function to be called when a message is received on a subscribed topic
  reconnect(); // Ensure we're connected to MQTT before proceeding
}


void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop(); // This should be called regularly to process incoming messages
  static bool lastLedState = !ledState; // Static variable to track the last LED state we handled
  // Check if an interrupt has occurred
  // Check if an interrupt has occurred and the ledState has been toggled
  if (ledState != lastLedState) {
    lastLedState = ledState; // Update the lastLedState to the new state
    digitalWrite(ledPin, ledState); // Update the LED
    client.publish(ledTopic, ledState ? "ON" : "OFF", true); // Publish new state, retain message
  }
}
