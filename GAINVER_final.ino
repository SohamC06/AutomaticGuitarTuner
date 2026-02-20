//final ver

#include "arduinoFFT.h"
#include <driver/i2s.h>
#include <ESP32Servo.h>

#define servopin 5

#define I2S_WS 4
#define I2S_SD 21
#define I2S_SCK 22

int choice = 1;
int powerOn;
int type = 0;
int serialCommand = 0;
double serialValue = 0;

float divide;

bool doneTuning = true;

float S6 = 82.41, S5 = 110, S4 = 146.83, S3 = 196, S2 = 246.94, S1 = 329.63;
float current;

Servo myservo;


#define SAMPLES 4096
#define SAMPLING_FREQUENCY 16000
#define MIN_THRESHOLD 7000

float vReal[SAMPLES];
float vImag[SAMPLES];
int32_t rawsample[SAMPLES];
size_t bytes_read;

void setup() {
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50); 
  myservo.attach(servopin);
  Serial.begin(115200);


  const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLING_FREQUENCY,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false
  };

  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}

void loop() {


  long sum_of_magnitudes = 0;
  i2s_read(I2S_NUM_0, rawsample, sizeof(rawsample), &bytes_read, 100);
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = (float)(rawsample[i]);
    vImag[i] = 0;

    sum_of_magnitudes += abs(vReal[i]);
  }

  float average_amplitude = (float)sum_of_magnitudes / SAMPLES;

  if (average_amplitude < MIN_THRESHOLD) {
    myservo.write(90);
    delay(500);   
    Serial.println("Entered Noise Gate");
  } else {
    ArduinoFFT<float> FFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);
    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude();
    float peak = FFT.majorPeak();

    switch (choice) {
      case 1:
        divide = S1;
        break;
      case 2:
        divide = S2;
        break;
      case 3:
        divide = S3;
        break;
      case 4:
        divide = S4;
        break;
      case 5:
        divide = S5;
        break;
      case 6:
        divide = S6;
        break;
      default:
        Serial.println("Invalid choice in divide assignment");
        break;
    }
    int harmonic = round(peak / divide);
    if (harmonic == 0) {
      return;
    } else {
      current = peak / harmonic;
    }

    float z = current - divide;
    int gain = choice>=4?300:100;
    int dynrot = (int)(abs(z)*gain);
    if(dynrot>300 && choice>=4)
    {
        dynrot = 400;
    }
    if(dynrot>100 && choice<4)
    {
      dynrot = 200;
    }
    if (doneTuning == false) {
      if (type == 0)  //acoustic
      {
        switch (choice)  // choice comes from blynk
        {
          case 6:
            if (current < S6)  //current less than reqd
            {
              if (current >= S6 - 0.4 && current <= S6)  //current in non audible tuning difference range (LESS)
              {
                Serial.println("6th String Tuned");
                doneTuning = true;
              } else if (current > S6 - 20)  // string distuned while current is less
              {
                myservo.write(40);
                delay(dynrot);
                myservo.write(90);
                delay(3000);
                Serial.println("Tuning up....");
                // to happen automatically
              }
            } else if (current > S6) {
              if (current >= S6 && current <= S6 + 0.4)  //current in non audible tuning difference range(MORE)
              {
                Serial.println("6th String Tuned");
                doneTuning = true;
              } else if (current < S6 + 20)  // string distuned while current is more
              {
                myservo.write(170);
                delay(dynrot);
                myservo.write(90);
                delay(3000);
                Serial.println("Tuning down...");
                
              }
            } else  //tuning exactly correct
            {
              Serial.println("6th String tuned");
              doneTuning = true;
            }
            break;

          case 5:
            if (current < S5)  //current less than reqd
            {
              if (current >= S5 - 0.5 && current <= S5)  //current in non audible tuning difference range (LESS)
              {
                Serial.println("5th String Tuned");
                doneTuning = true;
              } else if (current > S5 - 20)  // string distuned while current is less
              {
                myservo.write(40);
                delay(dynrot);
                myservo.write(90);
                delay(3000);
                Serial.println("Tuning up....");
                // to happen automatically
              }
            } else if (current > S5) {
              if (current >= S5 && current <= S5 + 0.5)  //current in non audible tuning difference range(MORE)
              {
                Serial.println("5th String Tuned");
                doneTuning = true;
              } else if (current < S5 + 20)  // string distuned while current is more
              {
                myservo.write(160);
                 delay(dynrot);
                myservo.write(90);
                 delay(3000);
                 Serial.println("Tuning down....");
                // to happen automatically
              }
            } else  //tuning exactly correct
            {
              Serial.println("5th String tuned");
              doneTuning = true;
            }
            break;

          case 4:
            if (current < S4)  //current less than reqd
            {
              if (current >= S4 - 0.43 && current <= S4)  //current in non audible tuning difference range (LESS)
              {
                Serial.println("4th String Tuned");
                doneTuning = true;
              } else if (current > S4 - 20)  // string distuned while current is less
              {
                Serial.println("Tuning up...");
                myservo.write(50);
                 delay(dynrot);
                myservo.write(90);
                 delay(3000);
                // to happen automatically
              }
            } else if (current > S4) {
              if (current >= S4 && current <= S4 + 0.43)  //current in non audible tuning difference range(MORE)
              {
                Serial.println("4th String Tuned");
                doneTuning = true;
              } else if (current < S4 + 20)  // string distuned while current is more
              {
                Serial.println("Tuning down....");
                myservo.write(145);
                 delay(dynrot);
                myservo.write(90);
                 delay(3000);
                 
                // to happen automatically
              }
            } else  //tuning exactly correct
            {
              Serial.println("4th String tuned");
              doneTuning = true;
            }
            break;

          case 3:
            if (current < S3)  //current less than reqd
            {
              if (current >= S3 - 0.57 && current <= S3)  //current in non audible tuning difference range (LESS)
              {
                Serial.println("3rd String Tuned");
                doneTuning = true;
              } else if (current > S3 - 20)  // string distuned while current is less
              {
                myservo.write(55);
                 delay(dynrot);
                Serial.println("Tuning up...");
                myservo.write(90);
                 delay(3000);
                // to happen automatically
              }
            } else if (current > S3) {
              if (current >= S3 && current <= S3 + 0.57)  //current in non audible tuning difference range(MORE)
              {
                Serial.println("3rd String Tuned");
                doneTuning = true;
              } else if (current < S3 + 20)  // string distuned while current is more
              {
                myservo.write(130);
                 delay(dynrot);
                Serial.println("Tuning Down...");
                myservo.write(90);
                 delay(3000);
                // to happen automatically
              }
            } else  //tuning exactly correct
            {
              Serial.println("3rd String tuned");
              doneTuning = true;
            }
            break;

          case 2:
            if (current < S2)  //current less than reqd
            {
              if (current >= S2 - 0.71 && current <= S2)  //current in non audible tuning difference range (LESS)
              {
                Serial.println("2nd String Tuned");
                doneTuning = true;
              } else if (current > S2 - 20)  // string distuned while current is less
              {
                Serial.println("Tuning up...");
                myservo.write(55);
                delay(dynrot);
                myservo.write(90);
                delay(3000);
                // to happen automatically
              }
            } else if (current > S2) {
              if (current >= S2 && current <= S2 + 0.71)  //current in non audible tuning difference range(MORE)
              {
                Serial.println("2nd String Tuned");
                doneTuning = true;
              } else if (current < S2 + 20)  // string distuned while current is more
              {
                Serial.println("Tuning Down...");
                myservo.write(125);
                 delay(dynrot);
                myservo.write(90);
                 delay(3000);
                // to happen automatically
              }
            } else  //tuning exactly correct
            {
              Serial.println("2nd String tuned");
              doneTuning = true;
            }
            break;

          case 1:
            if (current < S1)  //current less than reqd
            {
              if (current >= S1 - 0.95 && current <= S1)  //current in non audible tuning difference range (LESS)
              {
                Serial.println("1st String Tuned");
                doneTuning = true;
              } else if (current > S1 - 20)  // string distuned while current is less
              {
                Serial.println("Tuning Up...");
                myservo.write(55);
                delay(dynrot);
                myservo.write(90);
                delay(3000);
                
                // to happen automatically
              }
            } else if (current > S1) {
              if (current >= S1 && current <= S1 + 0.95)  //current in non audible tuning difference range(MORE)
              {
                Serial.println("1st String Tuned");
                doneTuning = true;
              } else if (current < S1 + 20)  // string distuned while current is more
              {
                Serial.println("Tuning Down");
                myservo.write(125);
                delay(dynrot);
                myservo.write(90);
                delay(3000);
                Serial.println("Tuning Down");
                // to happen automatically
              }
            } else  //tuning exactly correct
            {
              Serial.println("1st String tuned");
              doneTuning = true;
            }
            break;
          default:
            Serial.println("Invalid choice");
            break;
        }
      } else if (type == 1) {
        switch (choice)  // choice comes from blynk
        {
          case 6:
            if (current < S6)  //current less than reqd
            {
              if (current >= S6 - 0.24 && current <= S6)  //current in non audible tuning difference range (LESS)
              {
                Serial.println("6th String Tuned");
                doneTuning = true;
              } else if (current > S6 - 20)  // string distuned while current is less
              {
                Serial.println("Pre write print tune up");
                myservo.write(60);
                 delay(dynrot);
                myservo.write(90);
                 delay(3000);
                Serial.println("Tuning up for 6th string");
                // to happen automatically
              }
            } else if (current > S6) {
              if (current >= S6 && current <= S6 + 0.24)  //current in non audible tuning difference range(MORE)
              {
                Serial.println("6th String Tuned (MORE)");
                doneTuning = true;
              } else if (current < S6 + 20)  // string distuned while current is more
              {
                Serial.println("Pre write print tune down");
                myservo.write(120);
                 delay(dynrot);
                myservo.write(90);
                 delay(3000);
                Serial.println("Tuning down for 6th string");
                // to happen automatically
              }
            } else  //tuning exactly correct
            {
              Serial.println("6th String tuned(EXACT)");
              doneTuning = true;
            }
            break;

          case 5:
            if (current < S5)  //current less than reqd
            {
              if (current >= S5 - 0.32 && current <= S5)  //current in non audible tuning difference range (LESS)
              {
                Serial.println("5th String Tuned (LESS)");
                doneTuning = true;
              } else if (current > S5 - 20)  // string distuned while current is less
              {
                myservo.write(60);
                 delay(dynrot);
                myservo.write(90);
                 delay(3000);
                // to happen automatically
              }
            } else if (current > S5) {
              if (current >= S5 && current <= S5 + 0.32)  //current in non audible tuning difference range(MORE)
              {
                Serial.println("5th String Tuned (MORE)");
                doneTuning = true;
              } else if (current < S5 + 20)  // string distuned while current is more
              {
                myservo.write(120);
                 delay(dynrot);
                myservo.write(90);
                 delay(3000);
                // to happen automatically
              }
            } else  //tuning exactly correct
            {
              Serial.println("5th String tuned(EXACT)");
              doneTuning = true;
            }
            break;

          case 4:
            if (current < S4)  //current less than reqd
            {
              if (current >= S4 - 0.43 && current <= S4)  //current in non audible tuning difference range (LESS)
              {
                Serial.println("4th String Tuned (LESS)");
                doneTuning = true;
              } else if (current > S4 - 20)  // string distuned while current is less
              {
                myservo.write(60);
                 delay(dynrot);
                myservo.write(90);
                 delay(3000);
                // to happen automatically
              }
            } else if (current > S4) {
              if (current >= S4 && current <= S4 + 0.43)  //current in non audible tuning difference range(MORE)
              {
                Serial.println("4th String Tuned (MORE)");
                doneTuning = true;
              } else if (current < S4 + 20)  // string distuned while current is more
              {
                myservo.write(120);
                 delay(dynrot);
                myservo.write(90);
                 delay(3000);
                // to happen automatically
              }
            } else  //tuning exactly correct
            {
              Serial.println("4th String tuned(EXACT)");
              doneTuning = true;
            }
            break;

          case 3:
            if (current < S3)  //current less than reqd
            {
              if (current >= S3 - 0.57 && current <= S3)  //current in non audible tuning difference range (LESS)
              {
                Serial.println("3rd String Tuned (LESS)");
                doneTuning = true;
              } else if (current > S3 - 20)  // string distuned while current is less
              {
                myservo.write(60);
                 delay(dynrot);
                Serial.println("Tuning up");
                myservo.write(90);
                 delay(3000);
                // to happen automatically
              }
            } else if (current > S3) {
              if (current >= S3 && current <= S3 + 0.57)  //current in non audible tuning difference range(MORE)
              {
                Serial.println("3rd String Tuned (MORE)");
                doneTuning = true;
              } else if (current < S3 + 20)  // string distuned while current is more
              {
                myservo.write(120);
                 delay(dynrot);
                Serial.println("Tuning Down");
                myservo.write(90);
                 delay(3000);
                // to happen automatically
              }
            } else  //tuning exactly correct
            {
              Serial.println("3rd String tuned(EXACT)");
              doneTuning = true;
            }
            break;

          case 2:
            if (current < S2)  //current less than reqd
            {
              if (current >= S2 - 0.71 && current <= S2)  //current in non audible tuning difference range (LESS)
              {
                Serial.println("2nd String Tuned (LESS)");
                doneTuning = true;
              } else if (current > S2 - 20)  // string distuned while current is less
              {
                myservo.write(60);
                 delay(dynrot);
                myservo.write(90);
                 delay(3000);
                // to happen automatically
              }
            } else if (current > S2) {
              if (current >= S2 && current <= S2 + 0.71)  //current in non audible tuning difference range(MORE)
              {
                Serial.println("2nd String Tuned (MORE)");
                doneTuning = true;
              } else if (current < S2 + 20)  // string distuned while current is more
              {
                myservo.write(120);
                 delay(dynrot);
                myservo.write(90);
                 delay(3000);
                // to happen automatically
              }
            } else  //tuning exactly correct
            {
              Serial.println("2nd String tuned(EXACT)");
              doneTuning = true;
            }
            break;

          case 1:
            if (current < S1)  //current less than reqd
            {
              if (current >= S1 - 0.95 && current <= S1)  //current in non audible tuning difference range (LESS)
              {
                Serial.println("1st String Tuned (LESS)");
                doneTuning = true;
              } else if (current > S1 - 20)  // string distuned while current is less
              {
                Serial.println("Pre write tune up");
                myservo.write(60);
                 delay(dynrot);
                myservo.write(90);
                 delay(3000);
                Serial.println("Tuning Up");
                // to happen automatically
              }
            } else if (current > S1) {
              if (current >= S1 && current <= S1 + 0.95)  //current in non audible tuning difference range(MORE)
              {
                Serial.println("1st String Tuned (MORE)");
                doneTuning = true;
              } else if (current < S1 + 20)  // string distuned while current is more
              {
                Serial.println("Pre write tune down");
                myservo.write(120);
                 delay(dynrot);
                myservo.write(90);
                 delay(3000);
                Serial.println("Tuning Down");
                // to happen automatically
              }
            } else  //tuning exactly correct
            {
              Serial.println("1st String tuned(EXACT)");
              doneTuning = true;
            }
            break;
          default:
            Serial.println("Invalid choice");
            break;
        }
      }
    }
  }
  if (doneTuning) {
    if (Serial.available() > 0) {
        char receivedChar = Serial.read();


        if (isAlpha(receivedChar)) {
        serialCommand = receivedChar;


        String serialValueString = Serial.readStringUntil('\n');
        serialValue = serialValueString.toDouble();
      }
    }
    if (serialCommand != 0) {
      if (serialCommand == 's' && serialValue > 0 && serialValue < 7) {
        choice = (int)(serialValue);
        doneTuning = false;
        Serial.print("Changed string to ");
        Serial.println(serialValue);
      } else if (serialCommand == 't' && serialValue >= 0 && serialValue <= 1) {
        type = (int)serialValue;
        Serial.println(type?"Electric":"Acoustic");
      }else {
        Serial.println("Invalid command");
      }
      serialCommand = 0;
    }
    delay(500);
  }
}