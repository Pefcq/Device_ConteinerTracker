#include <Adafruit_MPU6050.h>
#include <TinyGPS++.h>
#include <Adafruit_Sensor.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include "secrets.h"
#include "WiFi.h"

#define GPS_BAUDRATE 9600
#define AWS_IOT_PUBLISH_TOPIC   "esp32/pub"

float lat, lng, alt;

Adafruit_MPU6050 mpu;
TinyGPSPlus gps;

WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

void setup() {
  // Inicializa a comunicação serial a 115200 bps
  Serial.begin(19200);
  Serial2.begin(GPS_BAUDRATE);

  // Aguarda até que as interfaces serial estejam disponíveis
  while (!Serial) {
    delay(10);
  }

  connectAWS();

  Serial.println(F("Sistema Inicializando..."));

  // Inicializa o MPU6050
  while (!mpu.begin()) {
    Serial.println(F("Falha ao encontrar o chip MPU6050"));
    delay(100);
  }

  // Configura detecção de movimento
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(5);
  mpu.setMotionDetectionDuration(40);                             
  mpu.setInterruptPinLatch(true);  // Mantém latched. Desligará quando reiniciado.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  delay(100);
}

void loop() {
  // Verifica o status de interrupção de movimento
  if (mpu.getMotionInterruptStatus()) {
    // Tenta obter a posição GPS
    GetPos();
    publishMessage();
  }

  // Mantenha a conexão MQTT viva
  client.loop();

  delay(10);
}

void GetPos() {
  // Verifica se há dados disponíveis no Serial2
  while (Serial2.available() > 0) {
    // Codifica os dados do GPS
    if (gps.encode(Serial2.read())) {
      if (gps.location.isValid()) {
        Serial.print(F("- latitude: "));
        Serial.println(gps.location.lat(), 6);  // Precisão de 6 casas decimais
        lat = gps.location.lat();

        Serial.print(F("- longitude: "));
        Serial.println(gps.location.lng(), 6);  // Precisão de 6 casas decimais
        lng = gps.location.lng();

        Serial.print(F("- altitude: "));
        if (gps.altitude.isValid()) {
          Serial.println(gps.altitude.meters(), 2);  // Precisão de 2 casas decimais
          alt = gps.altitude.meters();

        } else {
          Serial.println(F("INVALIDO"));
        }
      } else {
        Serial.println(F("- localizacao: INVALIDO"));
      }

      Serial.print(F("- GPS data e hora: "));
      if (gps.date.isValid() && gps.time.isValid()) {
        Serial.printf("%04d-%02d-%02d %02d:%02d:%02d\n", gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second());
      } else {
        Serial.println(F("INVALIDO"));
      }
    }
  }
}

void connectAWS()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
 
  Serial.println("Connecting to Wi-Fi");
 
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
 
  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);
 
  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  client.setServer(AWS_IOT_ENDPOINT, 8883);
 
  Serial.println("Connecting to AWS IoT");
 
  while (!client.connect(THINGNAME))
  {
    Serial.print(".");
    delay(100);
  }
 
  if (!client.connected())
  {
    Serial.println("AWS IoT Timeout!");
    return;
  }
 
  Serial.println("AWS IoT Connected!");
}

void publishMessage()
{
  StaticJsonDocument<200> doc;
  doc["latitude"] = lat;
  doc["longitude"] = lng;
  doc["altitude"] = alt;
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); // print to client

  Serial.print("Publicando mensagem: ");
  Serial.println(jsonBuffer);

  if (client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer)) {
    Serial.println("Mensagem publicada com sucesso!");
  } else {
    Serial.println("Falha ao publicar a mensagem!");
  }
}
