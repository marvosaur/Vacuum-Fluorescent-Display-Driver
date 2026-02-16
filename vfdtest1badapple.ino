#include <Arduino.h>
#include "video_data.h"

// --- CONFIGURATION ---
#define VFD_TX_PIN    D6 
#define VFD_RX_PIN    D7 
#define UART_BAUD     460800
#define FPS           25 //playback fps, not VFD fps

//AA and 55 are for sync, and then third byte is brightness.
uint8_t header[3] = {0xAA, 0x55, 127}; 
uint8_t currentFrame[512]; 

void setup() {
  Serial.begin(115200);
  //these are SEEED XIAO ESP32c3 pins...
  Serial1.begin(UART_BAUD, SERIAL_8N1, VFD_RX_PIN, VFD_TX_PIN);
  
  delay(1000);
}

void loop() {
  for (uint16_t i = 0; i < totalFrames; i++) {
    unsigned long startTime = micros();

    float val = (sin(millis() / 1000.0) + 1.0) / 2.0; 
    header[2] = (uint8_t)(10 + (240 * val));

    //load frame from big ass header with hardcoded bad apple
    memcpy_P(currentFrame, videoData[i], 512);

    //send header
    Serial1.write(header, 3);

    //send data
    Serial1.write(currentFrame, 512);
    Serial1.flush();
    
    //esp32 is funky and this smooths it out a little
    delayMicroseconds(3000); 

    //framerate
    long frameDelay = (1000000 / FPS) - (micros() - startTime);
    if (frameDelay > 0) delayMicroseconds(frameDelay);
    
    if (i % 50 == 0) {
      Serial.print("Frame: "); Serial.print(i);
      Serial.print(" | Brightness: "); Serial.println(header[2]);
    }
  }
  Serial.println("Looping Video...");
}