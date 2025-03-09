#include <Wire.h>
#include "MPU6050.h"
#include "arduinoFFT.h"
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <U8g2lib.h>

MAX30105 particleSensor;
MPU6050 mpu;
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

#define MAX_BRIGHTNESS 255
#define SAMPLES 128           // Must be a power of 2
#define SAMPLING_FREQ 1000    // Hz

static const unsigned char image_heart_bits[] U8X8_PROGMEM = {0x1c,0x1c,0x3e,0x3e,0x7e,0x3f,0xff,0x7f,0xff,0x7f,0xff,0x7f,0xff,0x7f,0xff,0x7f,0xff,0x3f,0xfe,0x3f,0xfc,0x1f,0xfc,0x1f,0xf8,0x0f,0xf0,0x07,0xe0,0x03,0xc0,0x01};
static const unsigned char image_menu_information_sign_bits[] U8X8_PROGMEM = {0xe0,0x03,0x18,0x0c,0x04,0x10,0x02,0x20,0x02,0x20,0x01,0x40,0x01,0x40,0x01,0x40,0x01,0x40,0x01,0x40,0x02,0x20,0x02,0x20,0x04,0x10,0x18,0x0c,0xe0,0x03};
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
double realData[SAMPLES];     // Acceleration data for FFT
double imagData[SAMPLES];     // Imaginary part (set to zero)
double vibrationData[SAMPLES];
int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; // Raw acceleration for magnitude calculation

ArduinoFFT<double> FFT = ArduinoFFT<double>();

void setup() {
    Serial.begin(115200);
    Wire.begin();
    u8g2.begin();  // Initialize the display
    u8g2.enableUTF8Print(); // Enable UTF-8 support
    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("MPU6050 not connected!");
        while (1);
    }
    Serial.println("MPU6050 Connected.");
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  while (Serial.available() == 0) ; //wait until user presses a key
  Serial.read();

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
}

void loop() {
    double sumSquares = 0; // For RMS calculation
    double maxAccel = 0;   // Peak acceleration

    // Collect acceleration data
    for (int i = 0; i < SAMPLES; i++) {
        int16_t ax, ay, az;
        mpu.getAcceleration(&ax, &ay, &az);

        // Convert raw data to m/s²
        double ax_ms2 = (ax / 16384.0) * 9.81;
        double ay_ms2 = (ay / 16384.0) * 9.81;
        double az_ms2 = (az / 16384.0) * 9.81;

        // Remove DC offset (gravity) for vibration analysis
        double accelVib = sqrt(ax_ms2 * ax_ms2 + ay_ms2 * ay_ms2 + (az_ms2 - 9.81) * (az_ms2 - 9.81));

        // Store acceleration for RMS & FFT
        realData[i] = accelVib;
        vibrationData[i] = accelVib;
        imagData[i] = 0; // FFT imaginary part

        // Update sum of squares for RMS
        sumSquares += accelVib * accelVib;

        // Track peak acceleration
        if (accelVib > maxAccel) {
            maxAccel = accelVib;
        }

        delayMicroseconds(1000000 / SAMPLING_FREQ); // Control sampling rate
    }

    // Compute RMS magnitude
    double rmsAccel = sqrt(sumSquares / SAMPLES);

    // Apply window function for FFT (Hamming)
    FFT.windowing(realData, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);

    // Perform FFT
    FFT.compute(realData, imagData, SAMPLES, FFT_FORWARD);

    // Convert to magnitude
    FFT.complexToMagnitude(realData, imagData, SAMPLES);

    // Find peak frequency
    double peakFreq = FFT.majorPeak(realData, SAMPLES, SAMPLING_FREQ);

    // Display results
    Serial.print("Peak Vibration Acceleration: ");
    Serial.print(maxAccel);
    Serial.println(" m/s²");

    Serial.print("RMS Vibration Acceleration: ");
    Serial.print(rmsAccel);
    Serial.println(" m/s²");

    Serial.print("Dominant Vibration Frequency: ");
    Serial.print(peakFreq);
    Serial.println(" Hz");

    delay(500);

      bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while (1)
  {
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      //digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      //send samples and calculation result to terminal program through UART
      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.print(irBuffer[i], DEC);

      Serial.print(F(", HR="));
      Serial.print(heartRate, DEC);

      Serial.print(F(", HRvalid="));
      Serial.print(validHeartRate, DEC);

      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);

      Serial.print(F(", SPO2Valid="));
      Serial.println(validSPO2, DEC);
    }

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }
  int heartRate = 78;
    int spo2 = 96;
    float temperature = 36.8;
    int battery = 75;
    String medReminder = "Med: 2PM";
    

    // Start Drawing
    u8g2.clearBuffer();
    
    // Display Heart Rate with Icon 
    u8g2.drawXBMP(6, 6, 15, 16, image_heart_bits);    u8g2.setFont(u8g2_font_ncenB12_tr);
   u8g2.setFont(u8g2_font_5x7_tr);
u8g2.drawStr(23, 17, "100");

u8g2.drawStr(5, 35, "spO2");

u8g2.drawStr(2, 56, "Tremors");

u8g2.drawFrame(32, 27, 57, 12);

u8g2.drawFrame(32, 47, 58, 12);

u8g2.drawBox(33, 28, 29, 10);

u8g2.drawBox(33, 48, 47, 11);

u8g2.drawXBMP(59, 3, 15, 15, image_menu_information_sign_bits);

u8g2.drawLine(0, 0, 0, 0);

u8g2.drawLine(66, 4, 66, 16);

u8g2.setFont(u8g2_font_4x6_tr);
u8g2.drawStr(59, 24, "MEDS");

u8g2.setFont(u8g2_font_5x7_tr);
u8g2.drawStr(78, 14, "2PM");

u8g2.drawStr(74, 14, ":");

u8g2.drawStr(91, 37, "50");

u8g2.drawStr(91, 56, "20");

u8g2.drawLine(0, 0, 0, 0);

u8g2.drawLine(55, 0, 55, 30);

u8g2.drawStr(38, 17, "BPM");

u8g2.drawStr(102, 37, "%");



    // Battery Indicator
    u8g2.drawFrame(110, 0, 15, 8);
    u8g2.drawBox(110, 0, battery / 7, 8);

    // Send Buffer to Display
    u8g2.sendBuffer();
    
    delay(1000);
}
