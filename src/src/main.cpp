#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <stdlib.h>
#include "UbidotsEsp32mqtt.h"
#include "data.h"
#include "DHT_U.h"
#include "Settings.h"
#include <TFT_eSPI.h>

#define BUTTON_LEFT 0        // btn activo en bajo
#define LONG_PRESS_TIME 3000 // 3000 milis = 3s
#define DHTPIN 27            // definicion del pin
#define DHTTYPE DHT11        // definir el tipo de dht

TFT_eSPI tft = TFT_eSPI();                         // se define el constructor de las funciones principales
DHT dht(DHTPIN, DHTTYPE);                          // constructor
const char *UBIDOTS_TOKEN = "";                    // Put here your Ubidots TOKEN
const char *PUBLISH_DEVICE_LABEL = "esp32";        // Put here your Device label to which data  will be published
const char *VARIABLE_LABEL1 = "temperatura";       // Put here your Variable label to which data  will be published
const char *VARIABLE_LABEL2 = "humedad";           // Put here your Variable label to which data  will be published
const char *SUBSCRIBE_DEVICE_LABEL = "esp32";      // Replace with the device label to subscribe to
const char *SUBSCRIBE_VARIABLE_LABEL = "switch1";  // Replace with the variable label to subscribe to
const char *SUBSCRIBE_VARIABLE_LABEL2 = "switch2"; // Replace with the variable label to subscribe to
const int PUBLISH_FREQUENCY = 5000;                // Update rate in millisecondsx
const char *VARIABLE_LABEL3 = "statesw1";          // Replace with the variable label to subscribe to
const char *VARIABLE_LABEL4 = "statesw2";          // Replace with the variable label to subscribe to

char tmp[12]; // arreglo de char para almacenar el float convertido

unsigned long timer;
uint8_t analogPin = 34;  // Pin used to read data from GPIO34 ADC_CH6.
const uint8_t LED = 33;  // Pin used to write data based on 1's and 0's coming from Ubidots
const uint8_t LED2 = 22; // Pin used to write data based on 1's and 0's coming from Ubidots
int switch1 = 0, switch2 = 0, statesw1 = 0, statesw2 = 0;
Ubidots ubidots(UBIDOTS_TOKEN);

char cadfija2[] = "/v2.0/devices/esp32/switch1/lv";
char *buffe = "this is a test string";
WebServer server(80);

Settings settings;
int lastState = LOW; // para el btn
int currentState;    // the current reading from the input pin
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;

void load404();
void loadIndex();
void loadFunctionsJS();
void restartESP();
void saveSettings();
bool is_STA_mode();
void AP_mode_onRst();
void STA_mode_onRst();
void detect_long_press();
void callback(char *topic, byte *payload, unsigned int length);

// Rutina para iniciar en modo AP (Access Point) "Servidor"
void startAP()
{
  WiFi.disconnect();
  delay(19);
  Serial.println("Starting WiFi Access Point (AP)");
  WiFi.softAP("Manuela_AP", "facil123");
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
}

// Rutina para iniciar en modo STA (Station) "Cliente"
void start_STA_client()
{
  WiFi.softAPdisconnect(true);
  WiFi.disconnect();
  delay(100);
  Serial.println("Starting WiFi Station Mode");
  WiFi.begin((const char *)settings.ssid.c_str(), (const char *)settings.password.c_str());
  WiFi.mode(WIFI_STA);

  int cnt = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    // Serial.print(".");
    if (cnt == 100) // Si después de 100 intentos no se conecta, vuelve a modo AP
      AP_mode_onRst();
    cnt++;
    Serial.println("attempt # " + (String)cnt);
  }

  WiFi.setAutoReconnect(true);
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());
  pressedTime = millis();
  // Rutinas de Ubidots
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
  ubidots.subscribeLastValue(SUBSCRIBE_DEVICE_LABEL, SUBSCRIBE_VARIABLE_LABEL);  // Insert the device and variable's Labels, respectively
  ubidots.subscribeLastValue(SUBSCRIBE_DEVICE_LABEL, SUBSCRIBE_VARIABLE_LABEL2); // Insert the device and variable's Labels, respectively

  timer = millis();
}

void setup()
{

  Serial.begin(115200);
  delay(2000);

  EEPROM.begin(4096);                 // Se inicializa la EEPROM con su tamaño max 4KB
  pinMode(BUTTON_LEFT, INPUT_PULLUP); // btn activo en bajo
  // codigo pantalla:
  tft.init();             // se incializa la pantalla
  tft.fillScreen(0x0000); // llena la pantalla con color negro
  // inicializacion sensor
  Serial.begin(115200); // inicicalizar la comunicacion serial
  Serial.println(F("DHTxx test!"));
  dht.begin();
  Serial.begin(115200);
  // settings.reset();
  settings.load(); // se carga SSID y PWD guardados en EEPROM
  settings.info(); // ... y se visualizan

  Serial.println("");
  Serial.println("starting...");

  if (is_STA_mode())
  {
    start_STA_client();
  }
  else // Modo Access Point & WebServer
  {
    startAP();

    /* ========== Modo Web Server ========== */

    /* HTML sites */
    server.onNotFound(load404);

    server.on("/", loadIndex);
    server.on("/index.html", loadIndex);
    server.on("/functions.js", loadFunctionsJS);

    /* JSON */
    server.on("/settingsSave.json", saveSettings);
    server.on("/restartESP.json", restartESP);

    server.begin();
    Serial.println("HTTP server started");
  }
}

void loop()
{
  if (is_STA_mode()) // Rutina para modo Station (cliente Ubidots)
  {
    // codigo lectura sensor:
    delay(2000); // reatrdo de 2seg
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (isnan(h) || isnan(t))
    {
      Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }

    tft.drawString("Humedad:", 40, 10, 2);             // Y=VERTICAL 240 X=HORIZONTAL 135
    tft.drawString(dtostrf(h, 2, 2, tmp), 40, 40, 4);  // convierte el valor de float a string
    tft.drawString("Temperatura:", 20, 100, 2);        // Y=VERTICAL 240 X=HORIZONTAL 135
    tft.drawString(dtostrf(t, 2, 2, tmp), 40, 120, 4); // covierte el valor de float a string

    if (!ubidots.connected())
    {
      ubidots.reconnect();
      ubidots.subscribeLastValue(SUBSCRIBE_DEVICE_LABEL, SUBSCRIBE_VARIABLE_LABEL);  // Insert the device and variable's Labels, respectively
      ubidots.subscribeLastValue(SUBSCRIBE_DEVICE_LABEL, SUBSCRIBE_VARIABLE_LABEL2); // Insert the device and variable's Labels, respectively
    }
    if (abs(millis() - timer) > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds
    {
      // float value = analogRead(analogPin);
      ubidots.add(VARIABLE_LABEL1, t); // Insert your variable Labels and the value to be sent
      ubidots.add(VARIABLE_LABEL2, h); // Insert your variable Labels and the value to be sent
      ubidots.publish(PUBLISH_DEVICE_LABEL);
      ubidots.add(VARIABLE_LABEL3, statesw1); // Insert your variable Labels and the value to be sent
      ubidots.add(VARIABLE_LABEL4, statesw2);
      timer = millis();
    }
    if (switch1 == 1)
    {
      digitalWrite(LED, HIGH);
      tft.fillCircle(40, 200, 20, TFT_GREEN); // se dibuja el primer circulo- switch
    }
    else
    {
      digitalWrite(LED, LOW);
      tft.fillCircle(40, 200, 20, TFT_DARKGREY); //(X,Y,radio,color)
    }

    // Switch 2
    if (switch2 == 1)
    {
      digitalWrite(LED2, HIGH);
      tft.fillCircle(90, 200, 20, TFT_BLUE); // se dibuja el segundo circulo- switch
    }
    else
    {
      digitalWrite(LED2, LOW);
      tft.fillCircle(90, 200, 20, TFT_DARKGREY); //(X,Y,radio,color)
    }
    ubidots.loop();
  }
  else // rutina para AP + WebServer
    server.handleClient();
  delay(10);
  detect_long_press();
}
// funciones para responder al cliente desde el webserver:
// load404(), loadIndex(), loadFunctionsJS(), restartESP(), saveSettings()

void load404()
{
  server.send(200, "text/html", data_get404());
}

void loadIndex()
{
  server.send(200, "text/html", data_getIndexHTML());
}

void loadFunctionsJS()
{
  server.send(200, "text/javascript", data_getFunctionsJS());
}

void restartESP()
{
  server.send(200, "text/json", "true");
  ESP.restart();
}

void saveSettings()
{
  if (server.hasArg("ssid"))
    settings.ssid = server.arg("ssid");
  if (server.hasArg("password"))
    settings.password = server.arg("password");

  settings.save();
  server.send(200, "text/json", "true");
  STA_mode_onRst();
}

// Rutina para verificar si ya se guardó SSID y PWD del cliente
// is_STA_mode retorna true si ya se guardaron
bool is_STA_mode()
{
  if (EEPROM.read(flagAdr))
    return true;
  else
    return false;
}

void AP_mode_onRst()
{
  EEPROM.write(flagAdr, 0);
  EEPROM.commit();
  delay(100);
  ESP.restart();
}

void STA_mode_onRst()
{
  EEPROM.write(flagAdr, 1);
  EEPROM.commit();
  delay(100);
  ESP.restart();
}

void detect_long_press()
{
  // read the state of the switch/button:
  currentState = digitalRead(BUTTON_LEFT);

  if (lastState == HIGH && currentState == LOW) // button is pressed
    pressedTime = millis();
  else if (lastState == LOW && currentState == HIGH)
  { // button is released
    releasedTime = millis();

    // Serial.println("releasedtime" + (String)releasedTime);
    // Serial.println("pressedtime" + (String)pressedTime);

    long pressDuration = releasedTime - pressedTime;

    if (pressDuration > LONG_PRESS_TIME)
    {
      Serial.println("(Hard reset) returning to AP mode");
      delay(500);
      AP_mode_onRst();
    }
  }

  // save the the last state
  lastState = currentState;
}
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  buffe = topic;
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
    if ((char)payload[0] == '1')
    {
      if (strcmp(cadfija2, topic) == 0)
      {
        switch1 = 1;
        statesw1 = 1;
      }
      else
      {
        switch2 = 1;
        statesw2 = 1;
      }
    }
    else
    {
      if (strcmp(cadfija2, topic) == 0)
      {
        switch1 = 0;
        statesw1 = 0;
      }
      else
      {
        switch2 = 0;
        statesw2 = 0;
      }
    }
  }
  Serial.println();
}
