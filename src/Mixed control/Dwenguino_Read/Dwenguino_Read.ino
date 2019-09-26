/*
  Read out analog pin
  
 */

#include <Dwenguino.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <Filter.h>
#include <math.h>

int loadcell_IO, loadcell_FP, loadcell_ED, loadcell_LU;
int filtered_IO, filtered_FP, filtered_ED, filtered_LU;

Moving_average maIO(30);
Moving_average maFP(30);
Moving_average maED(30);
Moving_average maLU(30);

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(1000000);
  initDwenguino();

  dwenguinoLCD.clear();
  dwenguinoLCD.print("MEASURING");
  dwenguinoLCD.setCursor(0,1);
  dwenguinoLCD.print("LOAD CELLS");
}

// the loop function runs over and over again forever
void loop() {  
  loadcell_IO = analogRead(A0);
  loadcell_FP = analogRead(A2);
  loadcell_ED = analogRead(A3);
  loadcell_LU = analogRead(A1);

  filtered_IO = round(maIO.filter(loadcell_IO));
  filtered_FP = round(maFP.filter(loadcell_FP));
  filtered_ED = round(maED.filter(loadcell_ED));
  filtered_LU = round(maLU.filter(loadcell_LU));

  Serial.print(filtered_IO);
  Serial.print("\t");
  Serial.print(filtered_FP);
  Serial.print("\t");
  Serial.println(filtered_ED);
//  Serial.print("\t");
//  Serial.println(filtered_LU);
  
}
  
