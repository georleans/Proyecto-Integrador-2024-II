#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MiCS6814-I2C.h>
#include <PMS.h>
#include <U8x8lib.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <pgmspace.h>

// Pines y configuraciones
#define RXD2 16
#define TXD2 17
#define PIN_CO 35
#define DEVICE "ESP32"
#define SEALEVELPRESSURE_HPA (1013.25)

enum State { INITX = 40, READ_SENSORS = 41, DISPLAYX = 42, MQTT_PUBLISH = 43, ERROR = 44 };
State currentState = INITX;

// Estructura para datos de sensores
struct SensorData {
    float temp, humid, pressure, altitude;
    float coPPM;
    uint16_t pm1_0, pm2_5, pm10;
    bool pmDataAvailable;
};

SensorData currentSensorData;
bool sensorConnected;

// Variables globales
QueueHandle_t sensorQueue;
HardwareSerial SerialPMS(2);
PMS pms(SerialPMS);
PMS::DATA data;
Adafruit_BME280 bme;
MiCS6814 sensor;
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);
WiFiClientSecure net;
PubSubClient client(net);

// Constantes para AWS IoT
const char AWS_IOT_PUBLISH_TOPIC[] = "AQXpert/test/Esp32";
const char WIFI_SSID[] = "SANON";
const char WIFI_PASSWORD[] = "sanon1234";
const char AWS_IOT_ENDPOINT[] = "a3is9f0m32ncyk-ats.iot.us-east-1.amazonaws.com";

// Certificados AWS
static const char AWS_CERT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";

static const char AWS_CERT_CRT[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
MIIDWTCCAkGgAwIBAgIUeXvgKJw9dkja3hs01jA+6jUYPXIwDQYJKoZIhvcNAQEL
BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g
SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTI0MTExMTIxMDky
NFoXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0
ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBANitdLNMu0BZ33gNvUex
QqVqhuVkpXPTFfuzi6UhawYATt084kHolQBhWwweEs4siIaFhMZ03en3OcntbRcy
qlrTD+fVhw9cAD2l4CqCkfUiqOnKkRdg0AJzwilzC9lVB+V/mLOmpDpGw1Xz+wbD
eEWFQmJnM12f2aVOV911rGQzypsH1q3610ZnCLwWWKvVWqS6Ttk5QNpOFX/hXGMB
ub/nKzczZd+7g33ttNXSdXsEiqIdMHKWLD3OabWozNtI+99cuysJBcD9macpsyUM
pX7HC4nFMNkKQwawKICf2pVjj2yZKy3HGyzfsd5cu9teZ9uYYtYqLHcfJgFzBDHh
yj0CAwEAAaNgMF4wHwYDVR0jBBgwFoAUk//mw3FPN7YFUsTyLffxLIuo3j4wHQYD
VR0OBBYEFPNv1Yj2Gb2yJvS/J3IiZqNfMpzSMAwGA1UdEwEB/wQCMAAwDgYDVR0P
AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQCJlfT9upWWK1XqoLvlxz2ZkuKp
RoS+B5mCpx8+2FSZC/jKbnd/zh0l4959Y45UMDteyKf03rzJG0JF0LWm30KQk00N
uhSN7yEF3HORmC8eMjCJ+BrlVt4OAhz8JyhTpsfzsBJ/IOCy09VY+w834DIFxgv1
IT2qK6Kr9Ah78F+roLQubRqxQmgl1oCSSemb1BER/QuKfZcQYLEK7qXNArcsBTJs
xIlpgXm/aI/ETgN7vgBBermUBsnUjgCvdoYfE5x26k/03WVX6jhUyss35OwyzF3w
CrS+aY+op63LIVEcJYEXVKHlM2WOgGPGZE8JawAssw3Egmg2SuJxJsSqWpMQ
-----END CERTIFICATE-----
)KEY";

static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
MIIEpAIBAAKCAQEA2K10s0y7QFnfeA29R7FCpWqG5WSlc9MV+7OLpSFrBgBO3Tzi
QeiVAGFbDB4SziyIhoWExnTd6fc5ye1tFzKqWtMP59WHD1wAPaXgKoKR9SKo6cqR
F2DQAnPCKXML2VUH5X+Ys6akOkbDVfP7BsN4RYVCYmczXZ/ZpU5X3XWsZDPKmwfW
rfrXRmcIvBZYq9VapLpO2TlA2k4Vf+FcYwG5v+crNzNl37uDfe201dJ1ewSKoh0w
cpYsPc5ptajM20j731y7KwkFwP2ZpymzJQylfscLicUw2QpDBrAogJ/alWOPbJkr
LccbLN+x3ly7215n25hi1iosdx8mAXMEMeHKPQIDAQABAoIBABor+Wie4A9Jy8BI
iJBcTNyGHX0fxwxSdtlMOpXbj33OE7iNXhbv7O960vCHwWW940+8WOlPvG5COmy1
jPO8Xu9Rhx5NoF6ukijgHE+GLfMaGKUEnrngK7/gE5fGGaxKpQWHf/5TJosTeQys
x6ltet2K1xXjuUSZxkQ9Tkelqwj/L5hYsKDaEbVPhNDUbuC0HYIbizh/mdCyWTd9
7BaqHQNIK3tb7f/L3XaN20QM5CzY9+/zs4EjrLYswW0Wihv2VGKpAO6Rwn8hJhLb
SMNORllnDL/3zP2abxI4E9THMUoV5MMSz5cF0mxalhJU62DngMVIus+kB1A24Od+
iJNJjFECgYEA65SBMt66t8739HqswoEE6bNUYcIGeLd/lsPmwpkcmJ4e8hbRQpIy
KaOWIDYNYgaTTnGl4qWrwVydUZoiKCGstD3r6EDbOduvcMEBTtGfKRfL04jkTfsA
DgyECyxPkpQQByhAXkAiWhLWnUVGQbpI85k/XkPQ8ehwfA/GswNymaMCgYEA63WB
kDoDXwBidm1alypUUKkupa75RZqWSqw5nJ2VU2g+fx/ii/ZrCllDuto8FAugCZSZ
3DJb7ydBqjkFT6ffyyftqvT1O4DOL38D7NguLR24JD5OStNze5Xye9zRcXzjzdSY
W89T/OyZG2yfBipN7fW4F9WF5CNflvsia0C8Cp8CgYB9pHl3sRSyfZ7+9AJ5xifh
oHQroziiX3Ob7CWBuflnaEWm5gZ7U8+PJ6Ek0s82bb1YxAIuLB+7g/Kfl+4Jq1QZ
2UCdYSHK7ODX6QgSingxiXB7BHKHKIwx8fjmoJ5b1b3qo0wZOGlZ0txFirHa0kig
KyaB1vd0XXzInf1icj9HswKBgQC/JVZ4ElB4SCSmxWGchDWMgbunr2MXWUaW+jeO
5/K4klDh/K16OCJvdUXho5bQIxvZizRYqN0vSawqFz4zizHHh/OOLGwE0S0RK8iw
9lKON6ksaxQ99Gi0tGb0nK++tMduah8BsU8abaxhsg1rAN7gxfCcELAwI3NPazJm
5VeCOwKBgQCCMqeyyO81fqbDrWRDR6X8OW6ffhoZ9El68cxJjZo9+atEIhVNvCGx
tq0XFjVpTiUxpoWD/ARP/riS7a6yi0eGgkoddQt27sKpx8O0NbRpBaugxNCQj/1C
mXj7Ic9KV7oYONLW43EM5DBQDy3s+bLZ2dGCzpHlIw9st4ZmRlC4AQ==
-----END RSA PRIVATE KEY------
)KEY";

// Filtros de promedio móvil
const int FILTER_WINDOW_SIZE = 5;
float tempBuffer[FILTER_WINDOW_SIZE] = {0};
float humidBuffer[FILTER_WINDOW_SIZE] = {0};
float pressureBuffer[FILTER_WINDOW_SIZE] = {0};
float coPPMBuffer[FILTER_WINDOW_SIZE] = {0};
int bufferIndex = 0;

// Prototipos
void setupAWS();
void connectAWS();
void publishToAWS(SensorData data);
void readSensors();
void displayData(SensorData data);
void handleError();
float applyMovingAverage(float* buffer, float newValue);
int getWiFiSignalStrength();

void setup() {
    Serial.begin(115200);
    pinMode(PIN_CO, INPUT);

    // Configuración OLED
    u8x8.begin();
    u8x8.setPowerSave(0);
    u8x8.setFont(u8x8_font_chroma48medium8_r);

    // Mensaje inicial en pantalla
    u8x8.clearDisplay();
    u8x8.println("Iniciando...");

    // Configurar sensores
    SerialPMS.begin(9600, SERIAL_8N1, RXD2, TXD2);
    pms.passiveMode();
    sensorConnected = sensor.begin();
    if (sensorConnected) {
        sensor.powerOn();
    }
    if (!bme.begin(0x76)) {
        Serial.println("¡No se encontró un sensor BME280 válido!");
        while (1);
    }

    // Configurar WiFi y AWS
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi conectado");

    setupAWS();

    // Crear cola para datos
    sensorQueue = xQueueCreate(1, sizeof(SensorData));
}

void loop() {
    switch (currentState) {
        case INITX:
            u8x8.println("Sistema inicializado");
            currentState = READ_SENSORS;
            break;

        case READ_SENSORS:
            readSensors();
            currentState = DISPLAYX;
            break;

        case DISPLAYX:
            displayData(currentSensorData);
            currentState = MQTT_PUBLISH;
            break;

        case MQTT_PUBLISH:
            publishToAWS(currentSensorData);
            currentState = READ_SENSORS;
            break;

        case ERROR:
            handleError();
            break;
    }

    if (!client.connected()) {
        connectAWS();
    }
    client.loop();
}

void setupAWS() {
    net.setCACert(AWS_CERT_CA);
    net.setCertificate(AWS_CERT_CRT);
    net.setPrivateKey(AWS_CERT_PRIVATE);
    client.setServer(AWS_IOT_ENDPOINT, 8883);
    connectAWS();
}

void connectAWS() {
    u8x8.clearDisplay();
    u8x8.println("Conectando a AWS...");
    while (!client.connected()) {
        if (client.connect(DEVICE)) {
            Serial.println("Conectado a AWS IoT");
            u8x8.println("AWS conectado");
        } else {
            delay(2000);
        }
    }
}

void readSensors() {
    currentSensorData.temp = applyMovingAverage(tempBuffer, bme.readTemperature());
    currentSensorData.humid = applyMovingAverage(humidBuffer, bme.readHumidity());
    currentSensorData.pressure = applyMovingAverage(pressureBuffer, bme.readPressure() / 100.0F);
    currentSensorData.altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

    int coValue = analogRead(PIN_CO);
    currentSensorData.coPPM = map(coValue, 0, 4095, 0, 100);  // Conversión personalizada

    pms.wakeUp();
    delay(3000);
    if (pms.readUntil(data)) {
        currentSensorData.pm1_0 = data.PM_AE_UG_1_0;
        currentSensorData.pm2_5 = data.PM_AE_UG_2_5;
        currentSensorData.pm10 = data.PM_AE_UG_10_0;
        currentSensorData.pmDataAvailable = true;
    } else {
        currentSensorData.pmDataAvailable = false;
    }
    pms.sleep();
}

void displayData(SensorData data) {
    u8x8.clearDisplay();
    u8x8.setCursor(0, 0);
    u8x8.printf("Senal WiFi: %ddBm", getWiFiSignalStrength());
    u8x8.setCursor(0, 1);
    u8x8.printf("Temp: %.2f C", data.temp);
    u8x8.setCursor(0, 2);
    u8x8.printf("Hum: %.2f %%", data.humid);
    u8x8.setCursor(0, 3);
    u8x8.printf("Pres: %.2f hPa", data.pressure);
    u8x8.setCursor(0, 4);
    u8x8.printf("CO: %.2f ppm", data.coPPM);
    if (data.pmDataAvailable) {
        u8x8.setCursor(0, 5);
        u8x8.printf("PM2.5: %u ug/m3", data.pm2_5);
    } else {
        u8x8.setCursor(0, 5);
        u8x8.println("PM2.5: --");
    }
}

void publishToAWS(SensorData data) {
    StaticJsonDocument<256> jsonDoc;
    jsonDoc["temperature"] = data.temp;
    jsonDoc["humidity"] = data.humid;
    jsonDoc["pressure"] = data.pressure;
    jsonDoc["altitude"] = data.altitude;
    jsonDoc["coPPM"] = data.coPPM;
    if (data.pmDataAvailable) {
        jsonDoc["pm2_5"] = data.pm2_5;
        jsonDoc["pm10"] = data.pm10;
    }

    char payload[256];
    serializeJson(jsonDoc, payload);
    client.publish(AWS_IOT_PUBLISH_TOPIC, payload);
}

void handleError() {
    u8x8.clearDisplay();
    u8x8.println("Error en el sistema!");
    delay(5000);
    ESP.restart();
}

float applyMovingAverage(float* buffer, float newValue) {
    buffer[bufferIndex] = newValue;
    float sum = 0;
    for (int i = 0; i < FILTER_WINDOW_SIZE; i++) {
        sum += buffer[i];
    }
    bufferIndex = (bufferIndex + 1) % FILTER_WINDOW_SIZE;
    return sum / FILTER_WINDOW_SIZE;
}

int getWiFiSignalStrength() {
    return WiFi.RSSI();
}
