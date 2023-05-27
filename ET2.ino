#include <Arduino.h>
#define FILTER_TAP_NUM 35

//PIN:
const uint32_t buttonRecordingPin = 48;  // Port D4 correspond à la broche PIO_PA29
const uint32_t buttonFrequencyPin = 44;
const uint32_t ledPin = 52;


enum freqState {
  f8k,
  f16k,
  f32k,
  f44k
};

//Recording:
bool isBtnRecordingPressed = false;
bool isRecording = false;
uint32_t currentTime;
uint32_t previousTime;

//Signal analysis:
int filterIndex = 0;
int16_t filterBuffer[FILTER_TAP_NUM];

//Frequency Choice
int samplingFrequency = 44100;
int frequencyState = 3;
bool isBtnFrequencyPressed = false;



static const int16_t filter_taps[FILTER_TAP_NUM] = {
  21546,
  66439,
  51719,
  -22005,
  -17977,
  25148,
  -7355,
  -12632,
  19402,
  -10536,
  -6001,
  18987,
  -19836,
  5683,
  19599,
  -48033,
  70052,
  216511,
  70052,
  -48033,
  19599,
  5683,
  -19836,
  18987,
  -6001,
  -10536,
  19402,
  -12632,
  -7355,
  25148,
  -17977,
  -22005,
  51719,
  66439,
  21546
};

// ADC features
const int bufferSize = 7500;
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

void setup() {
  Serial.begin(115200);
  setupADC();
  setupTimer();
  // Setup buttons and led
  pinMode(ledPin, OUTPUT);
  pinMode(buttonRecordingPin, INPUT);
  pinMode(buttonFrequencyPin, INPUT);

  digitalWrite(ledPin, HIGH);  // Led on
                               //previousTime = millis();
}

void loop() {
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
  uint32_t adcValue = ADC->ADC_CDR[7];
  Serial.println(adcValue);


  /* if (bufferFull) {
    for (int i = 0; i < bufferSize; i++) {
      Serial.print("i: ");
     Serial.println(i);
     Serial.println(adcBuffer[i]);
    }
    Serial.println("----------");
    ADC->ADC_RNPR = (unsigned long)adcBuffer;  // Adresse du prochain buffer à charger
    ADC->ADC_RNCR = (unsigned int)bufferSize;  // Nombre d'éléments du prochain buffer à charger
    bufferFull = false;
  }*/
  //Serial.println(samplingFrequency);
}
