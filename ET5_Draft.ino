#include <Arduino.h>
#define RIF10_F8_TAP_NUM 0
#define RIF15_F8_TAP_NUM 0
#define RIF20_F8_TAP_NUM 0
#define RIF10_F16_TAP_NUM 0
#define RIF15_F16_TAP_NUM 0
#define RIF20_F16_TAP_NUM 0
#define RIF10_F32_TAP_NUM 35
#define RIF15_F32_TAP_NUM 5
#define RIF20_F32_TAP_NUM 0
#define RIF10_F44_TAP_NUM 37
#define RIF15_F44_TAP_NUM 39
#define RIF20_F44_TAP_NUM 0


//PIN:
const uint32_t buttonRecordingPin = 48;  // Port D4 correspond à la broche PIO_PA29
const uint32_t buttonFrequencyPin = 44;
const uint32_t buttonRIFPin = 40;
const uint32_t ledPin = 52;
const uint32_t dacReaderPin = A8;


enum freqState {
  f8k,
  f16k,
  f32k,
  f44k
};

enum RIF {
  fc10k,
  fc15k,
  fc20k,
  rifOff
};

//Recording:
bool isBtnRecordingPressed = false;
bool isRecording = false;
uint32_t currentTime;
uint32_t previousTime;

//Signal analysis:
int filterIndex = 0;
int16_t filterBuffer10F8[RIF10_F8_TAP_NUM];
int16_t filterBuffer15F8[RIF15_F8_TAP_NUM];
int16_t filterBuffer20F8[RIF20_F8_TAP_NUM];
int16_t filterBuffer10F16[RIF10_F16_TAP_NUM];
int16_t filterBuffer15F16[RIF15_F16_TAP_NUM];
int16_t filterBuffer20F16[RIF20_F16_TAP_NUM];
int16_t filterBuffer10F32[RIF10_F32_TAP_NUM];
int16_t filterBuffer15F32[RIF15_F32_TAP_NUM];
int16_t filterBuffer20F32[RIF20_F32_TAP_NUM];
int16_t filterBuffer10F44[RIF10_F44_TAP_NUM];
int16_t filterBuffer15F44[RIF15_F44_TAP_NUM];
int16_t filterBuffer20F44[RIF20_F44_TAP_NUM];


//Frequency Choice
int samplingFrequency = 44100;
int frequencyState = 3;
bool isBtnFrequencyPressed = false;

//RIF Choice
bool isRifOn = false;
int rifFrequency = 10000;
int rifState = 0;
bool isBtnRifPressed = false;


static const int16_t rif10_f8_taps[RIF10_F8_TAP_NUM] = {};

static const int16_t rif15_f8_taps[RIF15_F8_TAP_NUM] = {};

static const int16_t rif20_f8_taps[RIF20_F8_TAP_NUM] = {};

static const int16_t rif10_f16_taps[RIF10_F16_TAP_NUM] = {};

static const int16_t rif15_f16_taps[RIF15_F16_TAP_NUM] = {};

static const int16_t rif20_f16_taps[RIF20_F16_TAP_NUM] = {};

static const int16_t rif10_f32_taps[RIF10_F32_TAP_NUM] = {
  25,
  -1441,
  -2353,
  -253,
  1060,
  -624,
  -393,
  1036,
  -584,
  -670,
  1441,
  -619,
  -1371,
  2479,
  -649,
  -4137,
  9336,
  21185,
  9336,
  -4137,
  -649,
  2479,
  -1371,
  -619,
  1441,
  -670,
  -584,
  1036,
  -393,
  -624,
  1060,
  -253,
  -2353,
  -1441,
  25
};

static const int16_t rif15_f32_taps[RIF15_F32_TAP_NUM] = {
  -7586,
  292,
  16091,
  292,
  -7586,
};

static const int16_t rif20_f32_taps[RIF20_F32_TAP_NUM] = {};

static const int16_t rif10_f44_taps[RIF10_F44_TAP_NUM] = {
  22517,
  -89398,
  276415,
  -705232,
  1562866,
  -3111525,
  5681625,
  -9636743,
  15317456,
  -22974419,
  32690585,
  -44302484,
  57353459,
  -71102275,
  84577699,
  -96669741,
  106268469,
  -112437708,
  114597208,
  -112437708,
  106268469,
  -96669741,
  84577699,
  -71102275,
  57353459,
  -44302484,
  32690585,
  -22974419,
  15317456,
  -9636743,
  5681625,
  -3111525,
  1562866,
  -705232,
  276415,
  -89398,
  22517
};

static const int16_t rif15_f44_taps[RIF15_F44_TAP_NUM] = {

  1366,
  1413,
  -1206,
  -1343,
  239,
  -918,
  -752,
  450,
  -1035,
  -319,
  680,
  -1395,
  237,
  855,
  -2197,
  1367,
  967,
  -5065,
  8311,
  22849,
  8311,
  -5065,
  967,
  1367,
  -2197,
  855,
  237,
  -1395,
  680,
  -319,
  -1035,
  450,
  -752,
  -918,
  239,
  -1343,
  -1206,
  1413,
  1366

};

static const int16_t rif20_44_taps[RIF20_F44_TAP_NUM] = {};

void setupADC() {
  PMC->PMC_PCER1 |= PMC_PCER1_PID37;     // Enable the ADC peripheral
  ADC->ADC_MR = ADC_MR_PRESCAL(255)      // Set the prescaler to 255
                | ADC_MR_STARTUP_SUT64   // Set the startup time to 64 periods of ADC_CLK
                | ADC_MR_TRACKTIM(15)    // Set the tracking time to 15 periods of ADC_CLK
                | ADC_MR_SETTLING_AST3;  // Set the settling time to 17 periods of ADC_CLK
  ADC->ADC_CHER = ADC_CHER_CH7;          // Enable channel 7 (A0)
  // Configure Timer Counter 0 Channel 0 (TC0) for samplingFrequency
  PMC->PMC_PCER0 |= PMC_PCER0_PID27;  // Enable the TC0 peripheral
  TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_CPCTRG;
  // Set the clock source to TCLK4 (MCK / 128, 84 MHz / 128 = 656.25 kHz)
  // Enable the RC compare trigger
  // Set the RC value for a samplingFrequency Hz frequency
  TC0->TC_CHANNEL[0].TC_RC = 656250 / samplingFrequency - 1;
  // Enable the RC compare interrupt
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  // Enable the TC0_IRQn interrupt in the NVIC
  NVIC_EnableIRQ(TC0_IRQn);
}

void TC0_Handler() {
  // Read the status register to clear the interrupt flag
  TC0->TC_CHANNEL[0].TC_SR;
  // Start a new ADC conversion
  ADC->ADC_CR = ADC_CR_START;
}

int16_t applyRIF(int16_t inputValue, int rifStage) {
  int32_t outputValue = 0;
  int filterTapIndex = filterIndex;

  switch (rifStage) {
    case fc10k:
      {

        switch (frequencyState) {
          case f8k:
            {
              Serial.println("Impossible to apply a filter in this configuration, please change frequencies ");

              /*filterBuffer10F8[filterIndex] = inputValue;
              outputValue = 0;
              filterTapIndex = filterIndex;

              for (int i = 0; i < RIF10_F8_TAP_NUM; i++) {
                outputValue += rif10_f8_taps[i] * filterBuffer10F8[filterTapIndex];

                filterTapIndex--;
                if (filterTapIndex < 0) {
                  filterTapIndex = RIF10_F8_TAP_NUM - 1;
                }
              }

              filterIndex++;
              if (filterIndex >= RIF10_F8_TAP_NUM) {
                filterIndex = 0;
              }*/
              break;
            }
          case f16k:
            {
              Serial.println("Impossible to apply a filter in this configuration, please change frequencies ");
              /*
              filterBuffer10F16[filterIndex] = inputValue;
              outputValue = 0;
              filterTapIndex = filterIndex;

              for (int i = 0; i < RIF10_F16_TAP_NUM; i++) {
                outputValue += rif10_f16_taps[i] * filterBuffer10F16[filterTapIndex];

                filterTapIndex--;
                if (filterTapIndex < 0) {
                  filterTapIndex = RIF10_F16_TAP_NUM - 1;
                }
              }

              filterIndex++;
              if (filterIndex >= RIF10_F16_TAP_NUM) {
                filterIndex = 0;
              }*/
              break;
            }
          case f32k:
            {

              filterBuffer10F32[filterIndex] = inputValue;
              outputValue = 0;
              filterTapIndex = filterIndex;

              for (int i = 0; i < RIF10_F32_TAP_NUM; i++) {
                outputValue += rif10_f32_taps[i] * filterBuffer10F32[filterTapIndex];

                filterTapIndex--;
                if (filterTapIndex < 0) {
                  filterTapIndex = RIF10_F32_TAP_NUM - 1;
                }
              }

              filterIndex++;
              if (filterIndex >= RIF10_F32_TAP_NUM) {
                filterIndex = 0;
              }
              break;
            }

          default:
            {
              //BY default, we use the F44  FilterFrequency:
              filterBuffer10F44[filterIndex] = inputValue;
              outputValue = 0;
              filterTapIndex = filterIndex;

              for (int i = 0; i < RIF10_F44_TAP_NUM; i++) {
                outputValue += rif10_f44_taps[i] * filterBuffer10F44[filterTapIndex];

                filterTapIndex--;
                if (filterTapIndex < 0) {
                  filterTapIndex = RIF10_F8_TAP_NUM - 1;
                }
              }

              filterIndex++;
              if (filterIndex >= RIF10_F44_TAP_NUM) {
                filterIndex = 0;
              }
              break;
            }
        }



        break;
      }
    case fc15k:
      {
        switch (frequencyState) {
          case f8k:
            {
              Serial.println("Impossible to apply a filter in this configuration, please change frequencies ");
              /*
              filterBuffer15F8[filterIndex] = inputValue;
              outputValue = 0;
              filterTapIndex = filterIndex;

              for (int i = 0; i < RIF15_F8_TAP_NUM; i++) {
                outputValue += rif15_f8_taps[i] * filterBuffer15F8[filterTapIndex];

                filterTapIndex--;
                if (filterTapIndex < 0) {
                  filterTapIndex = RIF15_F8_TAP_NUM - 1;
                }
              }

              filterIndex++;
              if (filterIndex >= RIF15_F8_TAP_NUM) {
                filterIndex = 0;
              }*/
              break;
            }
          case f16k:
            {
              Serial.println("Impossible to apply a filter in this configuration, please change frequencies ");
              /*
              filterBuffer15F16[filterIndex] = inputValue;
              outputValue = 0;
              filterTapIndex = filterIndex;

              for (int i = 0; i < RIF15_F16_TAP_NUM; i++) {
                outputValue += rif15_f16_taps[i] * filterBuffer15F16[filterTapIndex];

                filterTapIndex--;
                if (filterTapIndex < 0) {
                  filterTapIndex = RIF15_F16_TAP_NUM - 1;
                }
              }

              filterIndex++;
              if (filterIndex >= RIF15_F16_TAP_NUM) {
                filterIndex = 0;
              }*/
              break;
            }
          case f32k:
            {
              Serial.println("Impossible to apply a filter in this configuration, please change frequencies ");

              /*
              filterBuffer15F32[filterIndex] = inputValue;
              outputValue = 0;
              filterTapIndex = filterIndex;

              for (int i = 0; i < RIF15_F32_TAP_NUM; i++) {
                outputValue += rif15_f32_taps[i] * filterBuffer15F32[filterTapIndex];

                filterTapIndex--;
                if (filterTapIndex < 0) {
                  filterTapIndex = RIF15_F32_TAP_NUM - 1;
                }
              }

              filterIndex++;
              if (filterIndex >= RIF15_F32_TAP_NUM) {
                filterIndex = 0;
              }*/
              break;
            }

          default:
            {
              //BY default, we use the F44 FilterFrequency:
              filterBuffer15F44[filterIndex] = inputValue;
              outputValue = 0;
              filterTapIndex = filterIndex;

              for (int i = 0; i < RIF15_F44_TAP_NUM; i++) {
                outputValue += rif15_f44_taps[i] * filterBuffer15F44[filterTapIndex];

                filterTapIndex--;
                if (filterTapIndex < 0) {
                  filterTapIndex = RIF15_F44_TAP_NUM - 1;
                }
              }

              filterIndex++;
              if (filterIndex >= RIF15_F44_TAP_NUM) {
                filterIndex = 0;
              }
              break;
            }
        }


        break;
      }
    case fc20k:
      {
        switch (frequencyState) {
          case f8k:
            {
              Serial.println("Impossible to apply a filter in this configuration, please change frequencies ");
              /*
              filterBuffer20F8[filterIndex] = inputValue;
              outputValue = 0;
              filterTapIndex = filterIndex;

              for (int i = 0; i < RIF20_F8_TAP_NUM; i++) {
                outputValue += rif20_f8_taps[i] * filterBuffer20F8[filterTapIndex];

                filterTapIndex--;
                if (filterTapIndex < 0) {
                  filterTapIndex = RIF20_F8_TAP_NUM - 1;
                }
              }

              filterIndex++;
              if (filterIndex >= RIF20_F8_TAP_NUM) {
                filterIndex = 0;
              }*/
              break;
            }
          case f16k:
            {
              Serial.println("Impossible to apply a filter in this configuration, please change frequencies ");
              /*
              filterBuffer20F16[filterIndex] = inputValue;
              outputValue = 0;
              filterTapIndex = filterIndex;

              for (int i = 0; i < RIF20_F16_TAP_NUM; i++) {
                outputValue += rif20_f16_taps[i] * filterBuffer20F16[filterTapIndex];

                filterTapIndex--;
                if (filterTapIndex < 0) {
                  filterTapIndex = RIF20_F16_TAP_NUM - 1;
                }
              }

              filterIndex++;
              if (filterIndex >= RIF20_F16_TAP_NUM) {
                filterIndex = 0;
              }*/
              break;
            }
          case f32k:
            {
              Serial.println("Impossible to apply a filter in this configuration, please change frequencies ");
              /*
              filterBuffer20F32[filterIndex] = inputValue;
              outputValue = 0;
              filterTapIndex = filterIndex;

              for (int i = 0; i < RIF20_F32_TAP_NUM; i++) {
                outputValue += rif20_f32_taps[i] * filterBuffer20F32[filterTapIndex];

                filterTapIndex--;
                if (filterTapIndex < 0) {
                  filterTapIndex = RIF20_F32_TAP_NUM - 1;
                }
              }

              filterIndex++;
              if (filterIndex >= RIF20_F32_TAP_NUM) {
                filterIndex = 0;
              }*/
              break;
            }

          default:
            {
              Serial.println("Impossible to apply a filter in this configuration, please change frequencies ");
              /*
              //BY default, we use the F8 FilterFrequency:
              filterBuffer20F8[filterIndex] = inputValue;
              outputValue = 0;
              filterTapIndex = filterIndex;

              for (int i = 0; i < RIF20_F8_TAP_NUM; i++) {
                outputValue += rif20_f8_taps[i] * filterBuffer20F8[filterTapIndex];

                filterTapIndex--;
                if (filterTapIndex < 0) {
                  filterTapIndex = RIF20_F8_TAP_NUM - 1;
                }
              }

              filterIndex++;
              if (filterIndex >= RIF20_F8_TAP_NUM) {
                filterIndex = 0;
              }*/
              break;
            }
        }

        break;
      }
    default:
      {
        Serial.println("RIF is OFF ");

        break;
      }
  }

  return outputValue >> 15;  // Adjust the shift value according to the number of filter taps used
}

void record() {
  // Read the ADC value from the A0 pin
  uint32_t adcValue = ADC->ADC_CDR[7];
  uint8_t value;

  switch (rifState) {
    case fc10k:
      {
        uint16_t filtervalue = applyRIF((uint16_t)adcValue, rifState);
        value = (uint8_t)(filtervalue >> 4);
        break;
      }
    case fc15k:
      {
        uint16_t filtervalue = applyRIF((uint16_t)adcValue, rifState);
        value = (uint8_t)(filtervalue >> 4);

        break;
      }
    case fc20k:
      {
        uint16_t filtervalue = applyRIF((uint16_t)adcValue, rifState);
        value = (uint8_t)(filtervalue >> 4);
        break;
      }
    default:
      {
        //RIF OFF:
        value = (uint8_t)adcValue;
        break;
      }
  }

  // Print the ADC value to the serial monitor
  //Serial.write(value);

  //--> ET3:
  //analogWrite(DAC1, adcValue);
  //uint32_t dacOutput = analogRead(dacReaderPin);
  //Serial.println(dacOutput);

  // Delay for a short period of time to control the sampling rate
  delay(1000 / samplingFrequency);
}

void setup() {
  Serial.begin(460800);
  setupADC();
  
  // Enable the timer counter and trigger it
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;

  // Setup buttons and led
  pinMode(ledPin, OUTPUT);
  pinMode(buttonRecordingPin, INPUT);
  pinMode(buttonFrequencyPin, INPUT);
  pinMode(buttonRIFPin, INPUT);
  pinMode(dacReaderPin, INPUT);

  previousTime = millis();
}

void loop() {
  currentTime = millis();
  // Lire l'état du bouton

  if (digitalRead(buttonRecordingPin) == HIGH && !isBtnRecordingPressed) {
    isBtnRecordingPressed = true;
  } else {
    isBtnRecordingPressed = false;
  }

  if (isBtnRecordingPressed && !isRecording) {
    previousTime = currentTime;
    isRecording = true;
  }

  if (isRecording) {
    if (currentTime - previousTime < 10000) {
      digitalWrite(ledPin, HIGH);  // Led on
      record();
    } else {
      digitalWrite(ledPin, LOW);  // Led off
      isRecording = false;
    }
  }

  //Frequency choice:
  int inputFrequencyChoice = digitalRead(buttonFrequencyPin);

  if (!isBtnFrequencyPressed && inputFrequencyChoice == 1) {
    frequencyState = (frequencyState + 1) % 4;
  }

  if (inputFrequencyChoice == 1) {
    isBtnFrequencyPressed = true;
  } else {
    isBtnFrequencyPressed = false;
  }

  switch (frequencyState) {
    case f8k:
      {
        samplingFrequency = 8000;
        break;
      }
    case f16k:
      {
        samplingFrequency = 16000;
        break;
      }
    case f32k:
      {
        samplingFrequency = 32000;
        break;
      }
    case f44k:
      {
        samplingFrequency = 44100;
        break;
      }
    default:
      {
        samplingFrequency = 44100;
        break;
      }
  }


  //RIF Choice:
  int rifInputValue = digitalRead(buttonRIFPin);

  if (!isBtnRifPressed && rifInputValue == 1) {
    rifState = (rifState + 1) % 4;
  }

  if (rifInputValue == 1) {
    isBtnRifPressed = true;
  } else {
    isBtnRifPressed = false;
  }

  switch (rifState) {
    case fc10k:
      {
        rifFrequency = 10000;
        isRifOn = true;
        break;
      }
    case fc15k:
      {
        rifFrequency = 15000;
        isRifOn = true;

        break;
      }
    case fc20k:
      {
        rifFrequency = 20000;
        isRifOn = true;
        break;
      }
    case rifOff:
      {
        rifFrequency = 10000;
        isRifOn = false;
        break;
      }
    default:
      {
        rifFrequency = 10000;
        isRifOn = false;
        break;
      }
  }
}
