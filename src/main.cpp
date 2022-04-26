#include <Arduino.h>
#include "I2SMEMSSampler.h"
#include "ADCSampler.h"
#include "classifier.h"
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

MAX30105 particleSensor;

const byte RATE_SIZE = 4; // Vidējas vērtības iegušanai
byte rates[RATE_SIZE];    // Masīvs ar sirds ritma rādītājiem
byte rateSpot = 0;
long lastBeat = 0; // Laiks kad reģistrēts pēdejais sitiens

float beatsPerMinute;
int beatAvg;

Eloquent::ML::Port::SVM clf;

ADCSampler *adcSampler = NULL;

// i2s konfigurācija no https://github.com/atomic14/esp32_audio
i2s_config_t adcI2SConfig = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_LSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0};

// Cik daudz vērtības nolasīt vienā reizē
const int SAMPLE_SIZE = 256;
// Mainīgais kura glabāt 4 sekunu audio datus
float soundSamples[256];
int nextSample = 0;

void adcWriterTask(void *param)
{
  I2SSampler *sampler = (I2SSampler *)param;
  int16_t *samples = (int16_t *)malloc(sizeof(uint16_t) * SAMPLE_SIZE);
  if (!samples)
  {
    Serial.println("Nevar iegūt datus");
    return;
  }
  while (true)
  {
    int samples_read = sampler->read(samples, SAMPLE_SIZE);
    if (nextSample == SAMPLE_SIZE)
    {
      continue;
    }
    float sample;
    float sum = 0;
    for (int i = 0; i < samples_read; i++)
    {
      sample = (samples[i] - 2048) / 4096.0;
      sum += pow(sample, 2);
    }
    soundSamples[nextSample] = sqrt(sum / samples_read);
    nextSample++;
  }
}

void setup()
{
  Serial.begin(115200);
  // Datu ieguve no analoga mikrofona MAX4466
  adcSampler = new ADCSampler(ADC_UNIT_1, ADC1_CHANNEL_7, adcI2SConfig);

  // Uzdevuma izveide mikroprocsera otrajam kodolam
  TaskHandle_t adcWriterTaskHandle;
  adcSampler->start();
  xTaskCreatePinnedToCore(adcWriterTask, "ADC Writer Task", 4096, adcSampler, 1, &adcWriterTaskHandle, 1);

  // Inicializē sirds ritma sensoru
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) // Izmanto noklusējuma I2C portu un 400kHz ātrumu
  {
    Serial.println("Sensors MAX30105 nav atrasts.");
  }
  particleSensor.setup();                    // Konfigurē oksimetrijas sensoru pēc noklusejuma konfigurācijas
  particleSensor.setPulseAmplitudeRed(0x0A); // Ieslēdz sarkano LED signālu, lai ziņotu, ka sensors strādā
  particleSensor.setPulseAmplitudeGreen(0);  // Atslēdz zaļo LED, kurš netiks izmantots

  if (!mlx.begin())
  {
    Serial.println("Sensors MLX netika atrasts.");
  }
}

void loop()
{
  if (nextSample == SAMPLE_SIZE)
  {
    Serial.println(clf.predictLabel(soundSamples));
    nextSample = 0;
  }
  long irValue = particleSensor.getIR();
  // Veic lasījumus tikai tad, kad tiek reģistrēta saskarse ar ādu
  if (irValue > 50000 && checkForBeat(irValue) == true)
  {
    // tiek reģistrēts sirds sitiens
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; // saglabā lasījumu masīvā
      rateSpot %= RATE_SIZE;                    // Wrap variable

      // Take average of readings
      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++)
      {
        beatAvg += rates[x];
      }
      beatAvg /= RATE_SIZE;

      Serial.println(beatAvg);
    }
  }
  Serial.print("Ambient = ");
  Serial.print(mlx.readAmbientTempC());
  Serial.print("*C\tObject = ");
  Serial.print(mlx.readObjectTempC());
  Serial.println("*C");
}