#include <HCSR04.h>
#include "KickSort.h"

HCSR04 frontUltra(4, 3); //initialisation class frontUltraSR04 (trig pin , echo pin)
HCSR04 leftUltra(12, 11);
HCSR04 rightUltra(8, 7);

int size = 3;
float frontul[3];
float leftul[3];
float rightul[3];

int frontdist = 0;
int leftdist = 0;
int rightdist = 0;


void setup()
{
    Serial.begin(9600);
}

void loop()
{

    for (int i = 0; i <= 2; i++) {
      int reading = frontUltra.dist();

      if (reading == 0) {
        reading = frontUltra.dist();
      }

      frontul[i] = reading;
    }

    delay(6);

    for (int i = 0; i <= 2; i++) {
      int reading = leftUltra.dist();

      if (reading == 0) {
        reading = leftUltra.dist();
      }

      leftul[i] = reading;
    }

    delay(6);

    for (int i = 0; i <= 2; i++) {
      int reading = rightUltra.dist();

      if (reading == 0) {
        reading = rightUltra.dist();
      }

      rightul[i] = reading;
    }

    delay(10);

    frontdist = frontul[0] >= frontul[1] ? frontul[0] : frontul[1];
    leftdist =  leftul[0] <= leftul[1] ? leftul[0] : leftul[1];
    rightdist = rightul[0] <= rightul[1] ? rightul[0] : rightul[1];

    String send = "";
    send += frontdist;
    send += ",";
    send += rightdist;
    send += ",";
    send += leftdist;

    Serial.println(send);
}
