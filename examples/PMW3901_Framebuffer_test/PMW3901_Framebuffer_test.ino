//"THE BEER-WARE LICENSE" (Revision 42): Arsenio Dev wrote this file. As long as you retain this notice you can do whatever you want with this stuff. 
//If we meet some day, and you think this stuff is worth it, you can buy me a beer in return.
#include "Optical_Flow_Sensor.h"

// param #1 digital pin 5 for chip select
// param #2 sensor type either PAA5100 or PMW3901 
Optical_Flow_Sensor flow(5, PMW3901);
char frame[35*35]; //array to hold the framebuffer


void setup() {
  Serial.begin(115200);
  if (!flow.begin()) {
    Serial.println("Initialization of the flow sensor failed");
    while(1) { }
  }
 
 flow.enableFrameBuffer(); 
 Serial.println("framebuffer init");
}


void loop() {
  flow.readFrameBuffer(frame);
  Serial.println("framebuffer read");
  int i,j,k;
  for(i=0, k=0; i<35; i++){ //i is Y pixel pos
    for(j=0; j<35; j++, k++){  //j is X pixel pos
      Serial.print(asciiart(frame[k]));
      Serial.print(' ');
      }
     Serial.println();
  }
  Serial.println();
}


char asciiart(int k){ //converter magic? Higher value shunts more right in char array 
  static char foo[] = "WX86*3I>!;~:,`. ";
  return foo[k>>4]; //return shunted from array value character
}
