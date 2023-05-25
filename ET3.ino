#include <Arduino.h>
#define FILTER_TAP_NUM 35

//PIN:
const uint32_t buttonRecordingPin = 48;  // Port D4 correspond à la broche PIO_PA29
const uint32_t buttonFrequencyPin = 44;
const uint32_t ledPin = 52;
const uint32_t dacReaderPin = A8;


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

int16_t applyFIRFilter(int16_t inputValue) {
  filterBuffer[filterIndex] = inputValue;
  int32_t outputValue = 0;
  int filterTapIndex = filterIndex;

  for (int i = 0; i < FILTER_TAP_NUM; i++) {
    outputValue += filter_taps[i] * filterBuffer[filterTapIndex];

    filterTapIndex--;
    if (filterTapIndex < 0) {
      filterTapIndex = FILTER_TAP_NUM - 1;
    }
  }

  filterIndex++;
  if (filterIndex >= FILTER_TAP_NUM) {
    filterIndex = 0;
  }

  return outputValue >> 15;  // Adjust the shift value according to the number of filter taps used
}

void record() {
  // Read the ADC value from the A0 pin
  uint32_t adcValue = ADC->ADC_CDR[7];
  // uint16_t filtervalue = applyFIRFilter((uint16_t)adcValue);
  // uint8_t value = (uint8_t)(filtervalue >> 4);
  // Print the ADC value to the serial monitor
  //Serial.write(adcValue);
  analogWrite(DAC1, adcValue);
  uint32_t dacOutput = analogRead(dacReaderPin);
  Serial.println(dacOutput);
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

  switch (frequencyState){
    case f8k:{
      samplingFrequency = 8000;
      break;
    }
    case f16k:{
      samplingFrequency = 16000;
      break;
    }
    case f32k:{
      samplingFrequency = 32000;
      break;
    }
    case f44k:{
      samplingFrequency = 44100;
      break;
    }
    defalut: {
      samplingFrequency = 44100;
      break;
    }
  }

}