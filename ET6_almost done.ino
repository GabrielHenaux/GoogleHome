#include <Arduino.h>
#include <Wire.h>
#include <arduinoFFT.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//Filters Macros
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

//Screen features:
#define SCREEN_WIDTH 128       // OLED display width, in pixels
#define SCREEN_HEIGHT 32       // OLED display height, in pixels
#define OLED_RESET -1          // (-1 shares reset pin Arduino)
#define SCREEN_ADDRESS_1 0x3C  //0x3C pour 128x32
Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//FFT Features:
#define SAMPLES 64
double vReal[SAMPLES];
double vImag[SAMPLES];
byte peak;
long maxpeak;
char buf[5];
arduinoFFT FFTC = arduinoFFT();

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


//Frequency Choice
int samplingFrequency = 44100;
int frequencyState = 3;
bool isBtnFrequencyPressed = false;

//RIF Choice
bool isRifOn = false;
int rifFrequency = 10000;
int rifState = rifOff;
bool isBtnRifPressed = false;

// ADC features
const int bufferSize = 500;
uint16_t adcBuffer[bufferSize];
volatile bool bufferFull = false;

// ADC configuration
void setupADC() {
  PMC->PMC_PCER1 |= PMC_PCER1_PID37;  // Enable the ADC peripheral

  ADC->ADC_CR = ADC_CR_SWRST;                           // Reset ADC
  ADC->ADC_PTCR = (ADC_PTCR_RXTDIS | ADC_PTCR_TXTDIS);  // Reset PDC transfer

  ADC->ADC_MR |= ADC_MR_TRGEN_EN             // Hardware trigger select
                 | ADC_MR_TRGSEL_ADC_TRIG1;  // Trigger by TIOA0
  ADC->ADC_MR = ADC_MR_PRESCAL(255)          // Set the prescaler to 255
                | ADC_MR_STARTUP_SUT64       // Set the startup time to 64 periods of ADC_CLK
                | ADC_MR_TRACKTIM(15)        // Set the tracking time to 15 periods of ADC_CLK
                | ADC_MR_SETTLING_AST3;      // Set the settling time to 17 periods of ADC_CLK
  ADC->ADC_CHER = ADC_CHER_CH7;              // Enable channel 7 (A0)

  ADC->ADC_IDR = ~(1u << 7);  // Désactiver toutes les interruptions sauf celle de CH7
  ADC->ADC_IER = (1u << 7);   // Activer l'interruption pour CH7

  // DMA configuration
  bufferFull = false;

  PMC->PMC_PCER1 |= PMC_PCER1_PID39;  // Active le périphérique PDC
  ADC->ADC_PTCR = ADC_PTCR_RXTDIS | ADC_PTCR_TXTDIS;
  ADC->ADC_RPR = (unsigned long)adcBuffer;  // Adresse de la mémoire de réception
  ADC->ADC_RCR = (unsigned int)bufferSize;  // Nombre d'éléments à recevoir
  ADC->ADC_RNPR = 0;                        // Adresse du prochain buffer à charger
  ADC->ADC_RNCR = 0;                        // Nombre d'éléments du prochain buffer à charger

  NVIC_EnableIRQ(ADC_IRQn);        // Activer l'interruption ADC dans le NVIC
  ADC->ADC_PTCR = ADC_PTCR_RXTEN;  // Activation du transfert de réception du PDC
}

// Timer TC0 channel 0 configuration
void setupTimer() {
  // Configure Timer Counter 0 Channel 0 (TC0) for Fe1
  PMC->PMC_PCER0 |= PMC_PCER0_PID27;                                       // Enable the TC0 peripheral
  TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_CPCTRG;  // Set the clock source to TCLK4 (MCK / 128, 84 MHz / 128 = 656.25 kHz)
  // Enable the RC compare trigger
  // Set the RC value for a Fe1 Hz frequency
  TC0->TC_CHANNEL[0].TC_RC = 656250 / samplingFrequency - 1;
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;  // Enable the timer counter and trigger it
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;                  // Enable the RC compare interrupt
  NVIC_EnableIRQ(TC0_IRQn);                                 // Enable the TC0_IRQn interrupt in the NVIC
}

// Interruption handler for the TCO timer
void TC0_Handler() {
  // Read the status register to clear the interrupt flag
  TC0->TC_CHANNEL[0].TC_SR;
  // Start a new ADC conversion
  ADC->ADC_CR = ADC_CR_START;
}

// Interruption handler for the ADC
void ADC_Handler() {
  ADC->ADC_CDR[7];
  unsigned long status = ADC->ADC_ISR;
  if (status & ADC_ISR_ENDRX) {
    bufferFull = true;
  }
}

// Display configuration
void setupDisplay1() {
  if (!display1.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS_1)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  display1.clearDisplay();
  display1.display();
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


              break;
            }
          case f16k:
            {
              Serial.println("Impossible to apply a filter in this configuration, please change frequencies ");

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

              break;
            }
          case f16k:
            {
              Serial.println("Impossible to apply a filter in this configuration, please change frequencies ");

              break;
            }
          case f32k:
            {
              Serial.println("Impossible to apply a filter in this configuration, please change frequencies ");

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

              break;
            }
          case f16k:
            {
              Serial.println("Impossible to apply a filter in this configuration, please change frequencies ");

              break;
            }
          case f32k:
            {
              Serial.println("Impossible to apply a filter in this configuration, please change frequencies ");

              break;
            }

          default:
            {
              Serial.println("Impossible to apply a filter in this configuration, please change frequencies ");

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

uint32_t filterApplication(uint32_t adcValue) {
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
  return (uint32_t)value;
}

void rifStatusController() {
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

void dacWriter(uint32_t adcValue) {
  analogWrite(DAC1, adcValue);                   // We write the value readed with the ADC on the DAC1 PIN
  uint32_t dacValue = analogRead(dacReaderPin);  //We read the output of the DAC after one resistor(perturbations problems);
  Serial.println(dacValue);                      //We write the value in the serial plotter
}

void samplingFrequencyController() {
  int inputFrequencyChoice = digitalRead(buttonFrequencyPin);

  if (!isBtnFrequencyPressed && inputFrequencyChoice == 1) {
    frequencyState = (frequencyState + 1) % 4;
  }

  if (inputFrequencyChoice == 1) {
    isBtnFrequencyPressed = true;
  } else {
    isBtnFrequencyPressed = false;
  }

  if (isBtnFrequencyPressed) {  // When we change sampling frequency
    switch (frequencyState) {
      case f8k:
        {
          samplingFrequency = 8000;
          setupTimer();
          break;
        }
      case f16k:
        {
          samplingFrequency = 16000;
          setupTimer();
          break;
        }
      case f32k:
        {
          samplingFrequency = 32000;
          setupTimer();
          break;
        }
      case f44k:
        {
          samplingFrequency = 44100;
          setupTimer();
          break;
        }
      default:
        {
          samplingFrequency = 44100;
          setupTimer();
          break;
        }
    }
  }
}

void drawFields() {
  int borderWidth = 2;

  display1.drawRect(0, 0, display1.width() / 2, display1.height() / 3, WHITE);
  display1.fillRect(borderWidth, borderWidth, (display1.width() / 2) - 2 * borderWidth, (display1.height() / 3) - 2 * borderWidth, BLACK);

  display1.drawRect(display1.width() / 2, 0, display1.width() / 2, display1.height() / 3, WHITE);
  display1.fillRect((display1.width() / 2) + borderWidth, borderWidth, (display1.width() / 2) - 2 * borderWidth, (display1.height() / 3) - 2 * borderWidth, BLACK);

  display1.drawRect(0, (display1.height() / 3), display1.width(), display1.height() / 1.4, WHITE);
  display1.fillRect(borderWidth, (display1.height() / 3) + borderWidth, (display1.width()) - 2 * borderWidth, (display1.height() / 1.4) - 2 * borderWidth, WHITE);
}

void drawFe(int Fe) {
  display1.setTextSize(1);               // Normal 1:1 pixel scale
  display1.setTextColor(SSD1306_WHITE);  // Draw white text
  display1.setCursor(2, 2);              // Start at top-left corner
  display1.print("Fe:");                 // Print Fe: on the display
  display1.print(Fe);
  display1.print("Hz");
}

void drawFc(int Fc) {
  display1.setTextSize(1);                            // Normal 1:1 pixel scale
  display1.setTextColor(SSD1306_WHITE);               // Draw white text
  display1.setCursor((display1.width() / 2) + 2, 2);  // Start at the middle for right field
  display1.print("Fc:");                              // Print Fc: on the display
  display1.print(Fc);
  display1.print("Hz");
}

void drawFFT() {
  int16_t x1;
  int16_t y1;
  uint16_t width;
  uint16_t height;

  display1.getTextBounds("FFT", 0, 0, &x1, &y1, &width, &height);

  // display on horizontal and vertical center
  display1.setTextSize(1);
  display1.setTextColor(SSD1306_WHITE);
  display1.setCursor(2, (SCREEN_HEIGHT - height) / 2);
  display1.print("FFT (Filtre RIF: ON)");  // text to display

  // La fonction loop ne fait rien d'autre que d'attendre une interruption
  for (byte i = 0; i < SAMPLES; i++) {
    vReal[i] = adcBuffer[i];
    vImag[i] = 0;
  }
  //FFTC.DCRemoval();
  FFTC.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFTC.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFTC.ComplexToMagnitude(vReal, vImag, SAMPLES);
  display1.fillRect(2, 12, display1.width() - 4, display1.height() - 13, BLACK);
  for (byte i = 0; i < SAMPLES / 2 - 3; i++) {
    peak = map(vReal[i + 2], 0, 1024, 0, 52);
    display1.fillRect(i * 4 + 6, abs(52 - peak) / 8 + 12, 3, peak / 2, WHITE);
  }
}

void setup() {
  Serial.begin(115200);

  //SETUPS:
  setupADC();
  setupTimer();
  setupDisplay1();

  // Setup buttons and led
  pinMode(ledPin, OUTPUT);
  pinMode(buttonRecordingPin, INPUT);
  pinMode(buttonFrequencyPin, INPUT);
  pinMode(dacReaderPin, INPUT);

  digitalWrite(ledPin, HIGH);  // Led on
}

void loop() {

  uint32_t adcValue = ADC->ADC_CDR[7];  //We read the value from the ADC
  adcValue = filterApplication(adcValue);
  //Serial.println(adcValue);

  //Frequency Choice:
  samplingFrequencyController();
  //RIF Choice:
  rifStatusController();
  //DAC :
  dacWriter(adcValue);

  //FFT Displaying:
  drawFields();
  drawFe(samplingFrequency);
  drawFc(rifFrequency);
  drawFFT();
  display1.display();


  if (bufferFull) {
    ADC->ADC_RNPR = (unsigned long)adcBuffer;  // Adresse du prochain buffer à charger
    ADC->ADC_RNCR = (unsigned int)bufferSize;  // Nombre d'éléments du prochain buffer à charger
    bufferFull = false;
  }
  //Serial.println(samplingFrequency);

  //currentTime = millis();
  // Lire l'état du bouton

  /*if (digitalRead(buttonRecordingPin) == HIGH && !isBtnRecordingPressed) {
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
    } else {
      digitalWrite(ledPin, LOW);  // Led off
      isRecording = false;
    }
  }*/
}