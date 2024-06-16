#include <Arduino.h>
#include <DHT.h>
#include <MQUnifiedsensor.h>
#include <BluetoothSerial.h>

#define DHTPIN 4 // Pino onde o DHT11 está conectado
#define DHTTYPE DHT11

#define FANPIN 5 // Pino PWM para controle da velocidade da fan
#define MQ_PIN 15 // Pino analógico onde o MQ-135 está conectado

DHT dht(DHTPIN, DHTTYPE);
BluetoothSerial SerialBT;

#define placa "ESP32"
#define Voltage_Resolution 3.3
#define ADC_Bit_Resolution 12 // O ESP32 tem resolução ADC de 12 bits por padrão
#define type "MQ-135"
#define ADC_Resolution pow(2, ADC_Bit_Resolution)
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, MQ_PIN, type);

int fanSpeed = 0; // Velocidade inicial da fan
bool manualMode = false; // Modo de operação: false = Automático, true = Manual

// Variáveis para armazenar leituras dos sensores
float temperature = 0.0;
float humidity = 0.0;
float airQuality = 0.0;

// Declaração das tarefas FreeRTOS
void TaskReadSensors(void *pvParameters);
void TaskControlFan(void *pvParameters);
void TaskBluetoothControl(void *pvParameters);

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32FanController"); // Nome do dispositivo Bluetooth
  dht.begin();

  // Configuração do PWM
  ledcSetup(0, 25000, 8); // Canal 0, 25kHz, resolução de 8 bits (25kHz é uma frequência comum para fans de 4 pinos)
  ledcAttachPin(FANPIN, 0); // Associa o pino FANPIN ao canal 0

  // Inicializa o sensor MQ-135
  MQ135.setRegressionMethod(1); // Define método de regressão
  MQ135.setA(110.47); // Define coeficiente 'a'
  MQ135.setB(-2.862); // Define coeficiente 'b'
  MQ135.init();
  Serial.println("Calibrando o sensor MQ-135...");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i++) {
    MQ135.update(); // Faz uma leitura do sensor
    calcR0 += MQ135.calibrate(2.0); // Calibra na concentração conhecida de 2 ppm
    delay(1000);
  }
  MQ135.setR0(calcR0 / 10); // Ajusta a média das leituras
  Serial.println("Calibração concluída!");

  // Criação das tarefas FreeRTOS
  xTaskCreate(TaskReadSensors, "ReadSensors", 2048, NULL, 1, NULL);
  xTaskCreate(TaskControlFan, "ControlFan", 2048, NULL, 1, NULL);
  xTaskCreate(TaskBluetoothControl, "BluetoothControl", 2048, NULL, 1, NULL);
}

void loop() {
  // O loop principal fica vazio, pois todas as operações são realizadas nas tarefas
}

void TaskReadSensors(void *pvParameters) {
  while (1) {
    // Leitura dos sensores
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();
    MQ135.update();
    airQuality = MQ135.readSensor(); // Leitura da qualidade do ar

    if (isnan(humidity) || isnan(temperature)) {
      Serial.println("Falha ao ler do sensor DHT!");
      SerialBT.println("Falha ao ler do sensor DHT!");
      fanSpeed = 0; // Se falhar a leitura, a velocidade da fan é zero
    }

    // Aguarda 2 segundos antes de ler novamente
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void TaskControlFan(void *pvParameters) {
  while (1) {
    if (!manualMode) { // Controle Automático
      // Controle automático baseado em temperatura e qualidade do ar
      if (temperature > 25 || airQuality > 9) {
        fanSpeed = 255; // Velocidade máxima
      } else if (temperature > 20 || airQuality > 5) {
        fanSpeed = 150; // Velocidade média
      } else {
        fanSpeed = 100; // Velocidade baixa
      }
      ledcWrite(0, fanSpeed); // Define a velocidade da fan
    }

    // Informações no Serial Monitor e no terminal Bluetooth
    String data = "Umidade: " + String(humidity) + "%  Temperatura: " + String(temperature) + "°C  Qualidade do ar: " + String(airQuality) + " PPM  Velocidade da fan: " + String(fanSpeed) + " (Modo: " + (manualMode ? "Manual" : "Automático") + ")";
    Serial.println(data);
    SerialBT.println(data);

    // Aguarda 2 segundos antes de atualizar novamente
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void TaskBluetoothControl(void *pvParameters) {
  while (1) {
    // Controle via Bluetooth
    if (SerialBT.available()) {
      String command = SerialBT.readStringUntil('\n');
      command.trim(); // Remove espaços em branco extras

      if (command.startsWith("FAN:")) {
        fanSpeed = command.substring(4).toInt();
        if (manualMode) {
          ledcWrite(0, fanSpeed); // Atualiza a velocidade da fan manualmente se em modo manual
        }
        String data = "Velocidade da fan ajustada manualmente para: " + String(fanSpeed);
        Serial.println(data);
        SerialBT.println(data);
      } else if (command.equalsIgnoreCase("MODE:MANUAL")) {
        manualMode = true;
        String data = "Modo Manual Ativado";
        Serial.println(data);
        SerialBT.println(data);
      } else if (command.equalsIgnoreCase("MODE:AUTO")) {
        manualMode = false;
        String data = "Modo Automático Ativado";
        Serial.println(data);
        SerialBT.println(data);
      } else {
        String data = "Comando desconhecido: " + command;
        Serial.println(data);
        SerialBT.println(data);
      }
    }

    // Aguarda 100 ms antes de checar novamente
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
