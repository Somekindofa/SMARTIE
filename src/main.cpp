#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <ArduinoJson.h>

const char* ssid = "Livebox-6A10";
const char* password = "xf3wgEcRLu6Gf7bLkV";
const char* mqtt_server = "192.168.1.14";
const char* ledTopic = "home/esp/led";

const int ledPin = LED_BUILTIN;                        // Replace with the actual GPIO pin connected to the LED
const int buttonPin = D1;                     // Replace with the actual GPIO pin connected to SW
volatile bool ledState = false;               // This will keep track of the LED state
volatile unsigned long lastDebounceTime = 0;  // Timestamp to track the last interrupt event
unsigned long debounceDelay = 20;             // Debounce delay in milliseconds

const int clkPin = D2; // Replace with the actual GPIO pin connected to CLK
const int dtPin = D3;  // Replace with the actual GPIO pin connected to DT
int counter = 0; // This will keep track of the encoder position
int lastEncoded = 0;

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
  pinMode(clkPin, INPUT_PULLUP);
  pinMode(dtPin, INPUT_PULLUP);

  counter = 0;

  digitalWrite(ledPin, LOW);  // Turn the LED off
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

  int MSB = digitalRead(clkPin); // MSB = most significant bit
  int LSB = digitalRead(dtPin); // LSB = least significant bit

  int encoded = (MSB << 1) | LSB; // converting the 2 pin value to single number
  int sum = (lastEncoded << 2) | encoded; // adding it to the previous encoded value

  static bool notchTurned = false; // Variable to track if a complete notch turn has occurred

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    if (!notchTurned) {
      counter++;
      notchTurned = true;
    }
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    if (!notchTurned) {
      counter--;
      notchTurned = true;
    }
  } else {
    notchTurned = false;
  }

  // If the counter has changed, publish the new value
  static int lastCounter = -1; // Static variable to store the last counter value

  if (counter != lastCounter) {
    // Create JSON object
    StaticJsonDocument<200> doc; // Adjust size based on your needs
    doc["encoder"] = counter;

    // Convert JSON object into a string
    char jsonOutput[200]; // Adjust size based on your needs
    serializeJson(doc, jsonOutput);

    // Publish JSON string to MQTT
    client.publish(ledTopic, jsonOutput);

    // Update lastCounter to the new value
    lastCounter = counter;
  }

  static bool lastLedState = !ledState; // Static variable to track the last LED state we handled
  // Check if an interrupt has occurred
  // Check if an interrupt has occurred and the ledState has been toggled
  if (ledState != lastLedState) {
    lastLedState = ledState; // Update the lastLedState to the new state
    digitalWrite(ledPin, ledState); // Update the LED
    client.publish(ledTopic, ledState ? "ON" : "OFF", true); // Publish new state, retain message
  }
}
