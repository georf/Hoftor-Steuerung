#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <credentials.h>

#define MASTER_1 D5 // 14
#define MASTER_2 D6 // 12
#define SECOND_1 D7 // 13
#define SECOND_2 D8 // 15
#define SWITCH D2
#define WARN_LIGHT_1 D0
#define WARN_LIGHT_2 D3
#define PRESSED_TIME 500
#define MQTT_STATUS_UPDATE_TIME 60000 // Every minute
#define MQTT_PUBLISH_CHANNEL "adebar/hoftor/state"
#define MQTT_SUBSCRIBE_CHANNEL_SWITCH "adebar/hoftor/switch"
#define MQTT_SUBSCRIBE_CHANNEL_OPEN "adebar/hoftor/open"
#define MQTT_SUBSCRIBE_CHANNEL_CLOSE "adebar/hoftor/close"

enum state
{
  unknown,
  opening,
  opened,
  closing,
  closed,
  running
};
state currentState = unknown;
state desiredState = unknown;
uint32_t lastStateChange = 0;
boolean lastMotorStates[4];
uint32_t lastMqttStatusUpdate = 0;
uint32_t switchPressedStart = 0;
boolean switchPressed = false;

WiFiClient espClient;
PubSubClient client(espClient);

void warnLightBlinking();
void readMotorStates(boolean *states);
void detectMotorStateChange();
void pressSwitch();
void desire(state wish);

void mqtt_reconnect();
void mqtt_callback(char *topic, byte *payload, unsigned int length);
void mqtt_send_status();

void setup(void)
{
  Serial.begin(115200);
  delay(10);
  Serial.println('\n');

  pinMode(MASTER_1, INPUT);
  pinMode(MASTER_2, INPUT);
  pinMode(SECOND_1, INPUT);
  pinMode(SECOND_2, INPUT);
  pinMode(SWITCH, OUTPUT);
  pinMode(WARN_LIGHT_1, OUTPUT);
  pinMode(WARN_LIGHT_2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // read current state
  readMotorStates(lastMotorStates);
  lastStateChange = millis();

  // wifi startup
  WiFi.setAutoReconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print('.');
    delay(100);
  }
  Serial.println("Connected to Wi-Fi sucessfully.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  client.setServer(mqttServer, mqttPort);
  client.setCallback(mqtt_callback);
}

void loop(void)
{
  if (!client.connected())
    mqtt_reconnect();
  client.loop();

  warnLightBlinking();
  detectMotorStateChange();

  if ((lastMqttStatusUpdate + MQTT_STATUS_UPDATE_TIME) < millis())
    mqtt_send_status();

  if (switchPressed && (switchPressedStart + PRESSED_TIME) < millis())
  {
    digitalWrite(SWITCH, LOW);
    switchPressed = false;
    Serial.println("Switch off");
  }
}

void mqtt_reconnect()
{
  if (!client.connected())
  {
    Serial.println("Reconnecting MQTT...");

    if (!client.connect("ESP8266Client", mqttUser, mqttPassword))
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
    }
    else
    {
      client.subscribe(MQTT_SUBSCRIBE_CHANNEL_SWITCH);
      client.subscribe(MQTT_SUBSCRIBE_CHANNEL_OPEN);
      client.subscribe(MQTT_SUBSCRIBE_CHANNEL_CLOSE);
      Serial.println("MQTT Connected...");
    }
  }
}

void mqtt_callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("MQTT: receive ");
  Serial.println(topic);
  if (strcmp(topic, MQTT_SUBSCRIBE_CHANNEL_SWITCH) == 0)
    pressSwitch();
  else if (strcmp(topic, MQTT_SUBSCRIBE_CHANNEL_OPEN) == 0)
    desire(opened);
  else if (strcmp(topic, MQTT_SUBSCRIBE_CHANNEL_CLOSE) == 0)
    desire(closed);
}

void pressSwitch()
{
  Serial.println("Switch on");
  digitalWrite(SWITCH, HIGH);
  switchPressed = true;
  switchPressedStart = millis();
}

void desire(state wish)
{
  if (currentState == wish)
    return;

  desiredState = wish;
  pressSwitch();
}

void warnLightBlinking()
{
  if (currentState == running || currentState == opening || currentState == closing)
  {
    if (millis() / 100 % 2 == 0)
    {
      digitalWrite(WARN_LIGHT_1, HIGH);
      digitalWrite(WARN_LIGHT_2, LOW);
      digitalWrite(LED_BUILTIN, LOW);
    }
    else
    {
      digitalWrite(LED_BUILTIN, HIGH);
      digitalWrite(WARN_LIGHT_1, LOW);
      digitalWrite(WARN_LIGHT_2, HIGH);
    }
  }
  else
  {
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(WARN_LIGHT_1, LOW);
    digitalWrite(WARN_LIGHT_2, LOW);
  }
}

void readMotorStates(boolean *states)
{
  states[0] = !digitalRead(MASTER_1);
  states[1] = !digitalRead(MASTER_2);
  states[2] = !digitalRead(SECOND_1);
  states[3] = !digitalRead(SECOND_2);
}

void detectMotorStateChange()
{
  uint32_t currentStateChange;
  uint32_t stateChangeDiff;
  boolean currentMotorStates[4];
  readMotorStates(currentMotorStates);

  if (currentMotorStates[0] == lastMotorStates[0] &&
      currentMotorStates[1] == lastMotorStates[1] &&
      currentMotorStates[2] == lastMotorStates[2] &&
      currentMotorStates[3] == lastMotorStates[3])
    return; // no change

  currentStateChange = millis();
  stateChangeDiff = currentStateChange - lastStateChange;

  if (stateChangeDiff < 200)
    return; // change not clear

  Serial.print(currentMotorStates[0]);
  Serial.print(currentMotorStates[1]);
  Serial.print(currentMotorStates[2]);
  Serial.println(currentMotorStates[3]);

  // opening step 1 (only 2 is on)
  if (!lastMotorStates[0] && !lastMotorStates[1] && !lastMotorStates[2] && !lastMotorStates[3] &&
      !currentMotorStates[0] && !currentMotorStates[1] && currentMotorStates[2] && !currentMotorStates[3])
  {
    currentState = opening;
  }
  // opening step 2 (2 was on and 0 is now on)
  else if (!lastMotorStates[0] && !lastMotorStates[1] && lastMotorStates[2] && !lastMotorStates[3] &&
           currentMotorStates[0] && !currentMotorStates[1] && currentMotorStates[2] && !currentMotorStates[3])
  {
    currentState = opening;
  }
  // opening finished (last state was opening and all off)
  else if (currentState == opening &&
           !currentMotorStates[0] && !currentMotorStates[1] && !currentMotorStates[2] && !currentMotorStates[3])
  {
    if (stateChangeDiff > 8000)
    {
      currentState = opened;
    }
    else
    {
      currentState = unknown;
    }
  }
  // closing step 1 (only 1 is on)
  else if (!lastMotorStates[0] && !lastMotorStates[1] && !lastMotorStates[2] && !lastMotorStates[3] &&
           !currentMotorStates[0] && currentMotorStates[1] && !currentMotorStates[2] && !currentMotorStates[3])
  {
    currentState = closing;
  }
  // closing step 2 (1 was on and 3 is now on)
  else if (!lastMotorStates[0] && lastMotorStates[1] && !lastMotorStates[2] && !lastMotorStates[3] &&
           !currentMotorStates[0] && currentMotorStates[1] && !currentMotorStates[2] && currentMotorStates[3])
  {
    currentState = closing;
  }
  // closing finished (last state was closing and all off)
  else if (currentState == closing &&
           !currentMotorStates[0] && !currentMotorStates[1] && !currentMotorStates[2] && !currentMotorStates[3])
  {
    if (stateChangeDiff > 8000)
    {
      currentState = closed;
    }
    else
    {
      currentState = unknown;
    }
  }
  // any motor is running
  else if (currentMotorStates[0] || currentMotorStates[1] || currentMotorStates[2] || currentMotorStates[3])
  {
    currentState = running;
  }
  // all motors are off
  else if (!currentMotorStates[0] || !currentMotorStates[1] || !currentMotorStates[2] || !currentMotorStates[3])
  {
    currentState = unknown;
  }

  lastMotorStates[0] = currentMotorStates[0];
  lastMotorStates[1] = currentMotorStates[1];
  lastMotorStates[2] = currentMotorStates[2];
  lastMotorStates[3] = currentMotorStates[3];
  lastStateChange = currentStateChange;

  if (desiredState != unknown)
  {
    if (desiredState == currentState)
      desiredState = unknown;
    else if (currentState == opened || currentState == closed || currentState == unknown)
      pressSwitch();
  }
  mqtt_send_status();
}

void mqtt_send_status()
{
  switch (currentState)
  {
  case unknown:
    client.publish(MQTT_PUBLISH_CHANNEL, "unknown");
    break;
  case opening:
    client.publish(MQTT_PUBLISH_CHANNEL, "opening");
    break;
  case opened:
    client.publish(MQTT_PUBLISH_CHANNEL, "opened");
    break;
  case closing:
    client.publish(MQTT_PUBLISH_CHANNEL, "closing");
    break;
  case closed:
    client.publish(MQTT_PUBLISH_CHANNEL, "closed");
    break;
  case running:
    client.publish(MQTT_PUBLISH_CHANNEL, "running");
    break;
  }
  lastMqttStatusUpdate = millis();
}
