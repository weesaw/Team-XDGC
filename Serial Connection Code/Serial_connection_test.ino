#include <stdint.h>

float data1[] = {1.0, 0.0, 0.0, 0.0};
float data2[] = {0.0, 1.0, 0.0, 0.0};
float data3[] = {0.0, 0.0, 1.0, 0.0};
float data4[] = {0.0, 0.0, 0.0, 1.0};
uint8_t i = 1;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);

}

void loop() {
  // put your main code here, to run repeatedly:
  /*if(i == 1)
  {
    Serial.write((uint8_t*)data1, sizeof(data1));
    i++;
  }
  else if(i ==2)
  {
    Serial.write((uint8_t*)data2, sizeof(data2));
    i++;
  }
  else if(i ==3)
  {
    Serial.write((uint8_t*)data3, sizeof(data3));
    i++;
  }
  else
  {
    Serial.write((uint8_t*)data4, sizeof(data4));
    i=1;  
  }
  Serial.write('\n');*/

  for(int j =0; j<5; j++)
    Serial.write((uint8_t*)data4, sizeof(data4));
  delay(200);

}
