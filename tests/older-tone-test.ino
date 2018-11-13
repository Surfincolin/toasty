// Need Tone library from rogue code
#include <Tone.h>

Tone tone1;
Tone tone2;
Tone tone3;

float tV1 = 0.0;
float tV2 = 40.0;

void setup()
{
 tone1.begin(8);
 tone2.begin(9);
//  tone3.begin(10);

 Serial.begin(9600);

}

void loop()
{
//  262 to 522 => 523 to 1045
 // tone1.play(262);
 // tone2.play(330);
 // tone3.play(392);

//    tV1++; tV2++;
//    int n1 =  tV1/100.0 * 260.0;
//    int n2 =  tV2/100.0 * 522.0;

    // Ease In
   int n1 = tV1*tV1/10000.0 * 260.0;
   int n2 = tV2*tV2/10000.0 * 522.0;
    //Ease Out
   // int n1 = tV1*(tV1-200.0)/-10000.0 * 260.0;
   // int n2 = tV2*(tV2-200.0)/-10000.0 * 522.0;

   tone1.play(262 + n1);
   tone2.play(523 + n2);

   if (tV1 >= 100.0) {
     tV1 = 0.0;
   }
   if (tV2 >= 100.0) {
     tV2 = 0.0;
   }
   delay(100);

}