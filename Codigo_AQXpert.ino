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
const char AWS_IOT_PUBLISH_TOPIC[] = "---------";
const char WIFI_SSID[] = "-------";
const char WIFI_PASSWORD[] = "------";
const char AWS_IOT_ENDPOINT[] = "----------";

// Certificados AWS
static const char AWS_CERT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
-----END CERTIFICATE-----
)EOF";

static const char AWS_CERT_CRT[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
-----END CERTIFICATE-----
)KEY";

static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
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
