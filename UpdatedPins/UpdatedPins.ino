#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#define START 12


LiquidCrystal_I2C lcd(0x27, 16, 2);

const int buttonPin1 =  12;
const int buttonPin2 = 14;
const int buttonResetPin = 13;
int buttonState1 = 0;
int buttonState2 = 0;         
int buttonResetState = 0; 

int counter = START;

void setup() {
  Serial.begin(9600);
  pinMode(buttonPin1, INPUT);
  pinMode(buttonPin2, INPUT);
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0 ,0);
  lcd.print(counter);
}

void loop() {
  //##################BUTTON 1##################################################
  if (debounceButton(buttonState1) == HIGH && buttonState1 == LOW) 
  {
    lcd.clear();
    counter++;
    lcd.print(counter);
    lcd.setCursor(1 ,0);
    buttonState1 = HIGH; 
   }
  else if (debounceButton(buttonState1) == LOW && buttonState1 == HIGH)
  {
    buttonState1 = LOW;
  }
   //##################BUTTON 2##################################################
  if (digitalRead(buttonPin2) == HIGH && buttonState2 == LOW) 
  {
    lcd.clear();
    counter--;
    lcd.print(counter);
    lcd.setCursor(1 ,0);
    buttonState2 = HIGH;
    //delay(10); 
   }
  else if (digitalRead(buttonPin2) == LOW && buttonState2 == HIGH)
  {
    buttonState2 = LOW;
    delay(10);
  }


     /////////////////////////////////BUTTON 3////////////////////////////////////////////////
  
  
  if (digitalRead(buttonResetPin ) == HIGH && buttonResetState == LOW) 
  {
    lcd.clear();
    lcd.setCursor(0 ,0);
    lcd.print("5 Sek halten");
    lcd.setCursor(0 ,1);
    lcd.print("fuer reset");
      for (int i = 0; i <= 5; i++) {
      lcd.setCursor(11 ,1);
      lcd.print(i);
      delay(1000);
      Serial.println(i);
        if (digitalRead(buttonResetPin) == LOW)
          break;
        if (i == 5){
          lcd.clear();
          lcd.setCursor(0 ,0);
          lcd.print("Reset");
          delay(1000);
          counter = START;
          setup();
          //Serial.println(i);
          }
  }
    buttonResetState = HIGH; 
   }
  else if (digitalRead(buttonResetPin) == LOW && buttonResetState == HIGH)
  {
    buttonResetState = LOW;
    delay(10);
  }



  
  //Serial.println(counter);
  }


boolean debounceButton(boolean state)
{
  boolean stateNow = digitalRead(buttonPin1);
  if(state!=stateNow)
  {
    delay(10);
    stateNow = digitalRead(buttonPin1);
  }
  return stateNow;
  
}
