//-------------------------------------MEMORY MAP----------------------------------------------------
/*
hours_lcd_1 : 0 
minutes_lcd_1 : 1
hours_lcd_2: 2 
minutes_lcd_2 : 3
//-----------------------
hours_lcd_timer_2_start: 4 
minutes_lcd_timer_start: 5
hours_lcd_timer_2_stop: 6
minutes_lcd_timer_2_stop:7
Mini_battery_voltage: 8
mini_battery_voltage_t2: 12
startup_voltage: 16
startloadvoltage_t2: 20
startuptimer_1 : 24
startuptimer_2: 26
//----------------------------
Runonbatteryvoltagemode: 28
upsmode: 29 
*/
#include <Arduino.h>
#include <LiquidCrystal.h>
#include <RTClib.h>
#include <Wire.h>
#include <EEPROM.h>
#include<avr/wdt.h>
//----------------------------LCD--------------------
//const int rs = 8, en =9, d4 = 10, d5 = 11, d6 = 12, d7 = 13;
const int rs = 8, en =9, d4 = 10, d5 = 11, d6 = 12, d7 = 13;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//-----------------------------------------Defines------------------------------
RTC_DS3231 rtc;
char t[32];
 //---------------------------------Defines-------------------------------------
 #define Relay_L_Solar 6
 #define Relay_L_Solar_2 7
 #define Set 2
 #define Decrement 1
 #define Increment 0
 #define AC_Available 3
 #define Exit A0
 #define Backlight 5

//-----------------------------------------Variables----------------------------
//unsigned short old_time_compare_pv,old_time_update_pv,old_time_screen_1=0,old_time_screen_2=0; // to async
char set_status=0;    //variable for the set button state
char txt[32];
char seconds_lcd_1=0,minutes_lcd_1=0,hours_lcd_1=0;
char seconds_lcd_2=0,minutes_lcd_2=0,hours_lcd_2=0;
char hours_lcd_timer2_start=0,hours_lcd_timer2_stop=0,seconds_lcd_timer2_start=0;
char minutes_lcd_timer2_start=0,minutes_lcd_timer2_stop=0,seconds_lcd_timer2_stop=0;
char Relay_State; // variable for toggling relay
char set_ds1307_minutes=0,set_ds1307_hours=0,set_ds1307_seconds=0,set_ds1307_day=0,set_ds1307_month=0;
uint16_t set_ds1307_year=2023;
char ByPassState=0;    //enabled is default 0 is enabled and 1 is disabled
float Battery_Voltage,PV_Voltage,Vin_PV,Vin_PV_Old=0,Vin_PV_Present=0;
char BatteryVoltageSystem=0; // to save the battery voltage system if it is 12v/24v/48v
unsigned int ADC_Value;   // adc value for battery voltage
unsigned int ADC_Value_PV;
double Vin_Battery;      //voltage of battery
float Mini_Battery_Voltage=0,Mini_Battery_Voltage_T2=0;     // for timer 1 and timer 2
char Timer_Enable=1;   // timer 1
char Timer_2_Enable=1; // timer 2
char Timer_3_Enable=1; //timer 3
char CorrectionTime_State=0;  // this function to solve the error when battery is low and timer didn't start because of the low battery
unsigned int High_Voltage=245;      //ac high voltag`e
unsigned int Low_Voltage=175;       // ad low voltage
char VoltageProtectorGood;
char BatteryGuardEnable=1;   // enabled is default
char VoltageProtectionEnable; // enable voltage protection on grid
char Error_Voltage=0;       //difference between voltage and reading voltage
float v; // ac voltage as global variable
char Saved_Voltage;     // volatge when user hits set
char Adjusted_Voltage; // voltage saved by user
char AcBuzzerActiveTimes=0;  //for not making buzzer always on
char AcBuzzerActive=0;  //  for controlling buzzer activation just for one time
char matched_timer_1_start,matched_timer_1_stop, matched_timer_2_start,matched_timer_2_stop;
char Old_Reg=0;
char SolarOnGridOff=0,SolarOffGridOn=0;
char SolarOnGridOff_2=0,SolarOffGridOn_2=0;
char Timer_isOn=0,Timer_2_isOn=0;
unsigned int Timer_Counter_2=0, Timer_Counter_3=0,Timer_Counter_4=0;
unsigned int Low_PV_Voltage=50;       // PV panels low voltage
bool Grid_Already_On=false;            // to not enter conditions as the grid is available
unsigned short old_timer_1=0,old_timer_2=0,temp=0;
unsigned int startupTIme_1=0,startupTIme_2=0;  // 25 seconds for load one to start up and 50 seconds for load 2 to startup
char updateScreen=0;
float arrayBatt[21];
float StartLoadsVoltage=0,StartLoadsVoltage_T2=0;
float BuzzerVoltage=0.1; // voltage added to mini voltage to start giving the alarm before loads switches off
unsigned short ReadMinutesMinusOldTimer_1=0;
unsigned short ReadMinutesMinusOldTimer_2=0;
unsigned int Timer_Counter_For_Grid_Turn_Off=0;
char RunTimersNowState=0;
unsigned int SecondsRealTime=0,CutSecondsRealTime_T1=0 , CutSecondsRealTime_T2=0 ;   // for holding reading seconds in real time for ac grid and startup timers
unsigned int SecondsRealTimePv_ReConnect_T1=0,SecondsRealTimePv_ReConnect_T2=0; // for reactive timers in sequence when timer switch off because off battery and wants to reload
unsigned int realTimeLoop=0;
bool RunWithOutBattery=true;
char const ButtonDelay=200;
char RunLoadsByBass=0;
char TurnOffLoadsByPass=0; // to turn off for error
char VoltageProtectorEnableFlag=1;
char every30MinutesInitScreen=0;
char initedScreenOnce=0;
unsigned int UpdateScreenTime=0,TimeToExitSetupProgram=0;
char SystemBatteryMode=0;
char EnterSetupProgram=0; // variable to detect if mcu in loop of setup program
unsigned int  ReadBatteryTime=0;
char RunOnBatteryVoltageMode=0;
bool UPSMode=0;       // i made ups mode and upo mode in same variable
char LoadsAlreadySwitchedOFF=0;
uint16_t Full_Seconds;
unsigned long currentTime = 0;
unsigned int CountSecondsRealTime=0;   // for secondsrealtime
unsigned int CountSecondsRealTimePv_ReConnect_T2=0,CountSecondsRealTimePv_ReConnect_T1=0;
unsigned int CountCutSecondsRealTime_T1=0,CountCutSecondsRealTime_T2=0; // time for cutting loads off 
//-------------------------------------------------------------------------------------------------------
//-----------------------------------Functions---------------------------------
void EEPROM_Load();
void Gpio_Init();
void Write_Time();
void Config();
void Config_Interrupts();
void LCD_Clear(unsigned short Row, unsigned short Start, unsigned short End);
void SetUpProgram();
void Timer_Delay_Config();
void SetTimerOn_1();
void SetTimerOff_1();
void SetTimerOn_2();
void SetTimerOff_2();
void SetDS1307_Time();
void SetDS1307Minutes_Program();
void SetDS1307Seconds_Program();
void TimerDelay();
void Read_Battery();
void SetLowBatteryVoltage();
void StoreBytesIntoEEprom();
void ReadBytesFromEEprom();
void SetTimer();   // set timer to be activated or not activated
void LowBatteryVoltageAlarm();
unsigned int ReadAC();  //read ac voltage
void CalculateAC();   //calculate ac voltage
void DisplayTimerActivation(); // to display if timer is enabled or disabled on LCD
void SetHighVoltage();
void SetLowVoltage();
void VoltageProtector(unsigned long voltage);
void EnableBatteryGuard();         // this function is for enabling or disable the battery protection
void EnableVoltageGuard();   // AC voltage Protection
void SetACVoltageError() ; // this function to adjust differents in the reading volt because of the error in the resistors
void GetVoltageNow(); // get AC voltage at time
void ToggleBuzzer();
void Start_Timer();
void Stop_Timer();
void ReadPV_Voltage();
void SetLowPV_Voltage();
void RestoreFactorySettings();
void EEPROM_FactorySettings(char period);
void Start_Timer_2_B();   // timer for updating screen
void Start_Timer_0_A();  // timer for pv voltage to shutdown
void Stop_Timer_0();
void Read_PV_Continues();  // to eep updating pv
void Startup_Timers();
void SetStartUpLoadsVoltage();
void RunTimersNow();
void TurnACLoadsByPassOn();
void RunTimersNowCheck();
void Watch_Dog_Timer_Enable();
void Watch_Dog_Timer_Disable();
void Write_Date(); // to set date of ds1307
void CheckForParams();
void LCD_ReConfig();
void Interrupt_INT0();
void Interrupt_INT1();
void Read_Time();
unsigned short ReadMinutes();
unsigned short ReadHours();
void AutoRunWithOutBatteryProtection();
void Timer_Seconds(); // timer for counting seconds 
void CheckWireTimeout();
void WDT_Disable();
//---------------------------------------------------------------------------------------
void Gpio_Init()
{
pinMode(6,OUTPUT);
pinMode(7,OUTPUT);  // Relay_L_Solar_2 set as output
pinMode(2,INPUT);  //Set as input  
pinMode(1,INPUT);  //Decrement as input   
pinMode(0,INPUT);  //Increment as input  
pinMode(3,INPUT);  //ac_available as input  
pinMode(A0,INPUT);  // set Exit as input 
pinMode(A1,INPUT);  // set Exit as input 
pinMode(5,OUTPUT);  // backlight
}
//-------------------------------------Config---------------------------------------------------
void Config()
{
Gpio_Init();
digitalWrite(Backlight,1);
lcd.begin(16,2);
lcd.clear();
lcd.noCursor();
lcd.setCursor(0,0);
lcd.print("   SLC V2.0.0   ");
delay(1500);
lcd.clear();
Wire.begin();
rtc.begin();
Wire.setWireTimeout(3000,true);   //refe : https://www.fpaynter.com/2020/07/i2c-hangup-bug-cured-miracle-of-miracles-film-at-11/
Wire.clearWireTimeoutFlag();
//EEPROM.begin();  
}
//----------------------------------Config Interrupts-----------------------------------
void Config_Interrupts()
{
attachInterrupt(digitalPinToInterrupt(2), Interrupt_INT0, RISING);   //Enter is pressed 
attachInterrupt(digitalPinToInterrupt(3), Interrupt_INT1, RISING);   // ac_avaiable
}
//---------------------------------Enter or Set Interrupt to turn backlight on---------------------------------
void Interrupt_INT0()
{
UpdateScreenTime=0; // if user pressed the button zero counter of dipslay backlight
digitalWrite(Backlight,1);
} 
//-------------------------------When Grid is Turned Off---------------------------------------------------------
void Interrupt_INT1()
{

 //-> functions for shutting down loads if there is no timers and grid is off
if(digitalRead(AC_Available)==1 && Timer_isOn==0  && RunLoadsByBass==0 && RunOnBatteryVoltageMode==0)
{
//AcBuzzerActiveTimes=0; // make buzzer va  riable zero to get activated once again
///old_timer_1=ReadMinutes();  // time must be updated after grid is off
SecondsRealTime=0;
CountSecondsRealTime=0;
digitalWrite(Relay_L_Solar,0);

}

if (digitalRead(AC_Available)==1 && Timer_2_isOn==0 && RunLoadsByBass==0 && RunOnBatteryVoltageMode==0)  // it must be   Timer_2_isOn==0    but because of error in loading eeprom value
{
CountSecondsRealTime=0;
SecondsRealTime=0;
digitalWrite(Relay_L_Solar_2,0);

}

if (digitalRead(AC_Available)==1 &&  RunLoadsByBass==0 && UPSMode==1 && LoadsAlreadySwitchedOFF==1)
{
LoadsAlreadySwitchedOFF=0;
SecondsRealTime=0;
SecondsRealTimePv_ReConnect_T1=0;
SecondsRealTimePv_ReConnect_T2=0;
CountSecondsRealTime=0;
CountSecondsRealTimePv_ReConnect_T1=0;
CountSecondsRealTimePv_ReConnect_T2=0;
digitalWrite(Relay_L_Solar,0);
digitalWrite(Relay_L_Solar_2,0);

}
//lcd.clear();
}
//-----------------------------------------Screen 1-------------------------------------------------
void Screen_1()
{
  //noInterrupts();
if (RunLoadsByBass==0) 
{
  lcd.setCursor(13,0);
  lcd.print("  ");
}   else
{
  lcd.setCursor(15,0);
  lcd.print("B");
}  
if (RunOnBatteryVoltageMode==0)
{
Read_Time();
}
else
{
  lcd.setCursor(0,0);
  lcd.print("Voltage Mode");

}
Read_Time();
Read_Battery();
CalculateAC();
//interrupts(); 
//LCD_CMD(_LCD_RETURN_HOME);    // to keep display in its location
}
//--------------------------------------Read Time-----------------------------------
void Read_Time()
{
DateTime now = rtc.now();
sprintf(t, "Time:%02d:%02d:%02d", now.hour(), now.minute(), now.second());
lcd.setCursor(0,0);
lcd.print(t);
}
//------------------------------------Read Battery------------------------------------
//--------------------------Read Battery Voltage--------------------------------
void Read_Battery()
{

float sum=0 , Battery[10];
char i=0;
ADC_Value=analogRead(A1);
Battery_Voltage=(ADC_Value *5.0)/1024.0;
//Vadc=Vin* (4.7K /100K) => Vin=(104.7/4.7k) * VADC
//100k*1.01=99K , 4.7K *1.01=4.653
///Vin_Battery=((10.5/0.5)*Battery_Voltage); // 0.3 volt error from reading
for ( i=0; i<10 ; i++)
{
Battery[i]=((10.5/0.5)*Battery_Voltage);
delay(50);
sum+=Battery[i];
}
Vin_Battery= sum/10.0 ;
lcd.setCursor(0,1);
lcd.print("V=");
dtostrf(Vin_Battery,4,1,txt);
lcd.setCursor(2,1);
lcd.print(txt);
if(digitalRead(AC_Available)==1)
{
LCD_Clear(1,6,15);
}
}
//----------------------------------------------Calculate AC----------------------------
void CalculateAC()
{

if (digitalRead(AC_Available) == 0 ) // in this if voltage protector is turned of no need for voltage read
{
  lcd.setCursor(6,1);
  lcd.print(" - Grid   ");

}

//VoltageProtector(v);
}
//-----------------------------------LCD Clear----------------------------------
void LCD_Clear(unsigned short Row, unsigned short Start, unsigned short End)
{
  unsigned short Column;
  for(Column=Start; Column<=End; Column++)
  {
    lcd.setCursor(Column,Row);
    lcd.print(" ");
  }
}
//-------------------------------Check For Set Program-----------------------------------------------
void CheckForSet()
{

if (digitalRead(Set)==0 && digitalRead(Exit)==0) 
{
delay(1500);
SetUpProgram();
}
}
//--------------------------------------Setup Program---------------------------------------------------
void SetUpProgram()
{
//Delay_ms(500);
if (digitalRead(Set)==0 && digitalRead(Exit)==0)
{
digitalWrite(Backlight,HIGH);
lcd.clear();
lcd.setCursor(0,0);
lcd.print("Setup Program");
delay(1500);
//---------------------------------Enter Programs ------------------------------
//-> enter setup mode and don't exit it until the user hit set button
while (digitalRead(Set)==1 )
{
//-> Enter First Timer Setting and test for exit button at every screen moving in and out
SetTimerOn_1();
if (digitalRead(Exit)==1 )   break;     //break out of the while loop
SetTimerOff_1();
if (digitalRead(Exit)==1 )   break;     //break out of the while loop
SetTimerOn_2();
if (digitalRead(Exit)==1 )   break;     //break out of the while loop
SetTimerOff_2();
if (digitalRead(Exit)==1 )   break;     //break out of the while loop
SetLowBatteryVoltage();// program 5 to set low battery voltage
if (digitalRead(Exit)==1 )   break;     //break out of the while loop
SetStartUpLoadsVoltage(); // program 15 to enable timer or disable
if (digitalRead(Exit)==1 )   break;     //break out of the while loop
Startup_Timers();
if (digitalRead(Exit)==1 )   break;     //break out of the while loop
SetDS1307_Time();    // program 10
if (digitalRead(Exit)==1 )   break;     //break out of the while loop
lcd.clear();
break;   // to break the while
} // end while
}    // end main if
}

//-----------------------------Setting Hour Timer 1-----------------------------
void SetTimerOn_1()
{

lcd.clear();
delay(500);
while (digitalRead(Set)==1 )
{
sprintf(txt,"[1] H:%02d-M:%02d",hours_lcd_1,minutes_lcd_1);
lcd.setCursor(0,0);
lcd.print(txt);
if (digitalRead(Exit)==1 )
{
break;     //break out of the while loop
}

//-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement)==1)
{
if (digitalRead(Increment)==1 )
{
delay(200);
minutes_lcd_1++;
}
if (digitalRead(Decrement)==1)
{
delay(200);
minutes_lcd_1--;
}
//-> perfect
if (minutes_lcd_1>59)    minutes_lcd_1=0;
if (minutes_lcd_1<0) minutes_lcd_1=0;
SecondsRealTimePv_ReConnect_T1=0;
Timer_isOn=0;
} // end while increment and decrement
} // end first while
//******************************************************************************
delay(500);     //read time for state
while (digitalRead(Set)==1 )
{
sprintf(txt,"[1] H:%02d-M:%02d",hours_lcd_1,minutes_lcd_1);
lcd.setCursor(0,0);
lcd.print(txt);
if (digitalRead(Exit)==1 )
{
break;     //break out of the while loop
}
 //-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement) == 1 )
{
if (digitalRead(Increment) == 1 )
{
delay(200);
hours_lcd_1++;
}
if (digitalRead(Decrement) == 1 )
{
delay(200);
hours_lcd_1--;
}

if  (hours_lcd_1>23) hours_lcd_1=0;
if  (hours_lcd_1<0) hours_lcd_1=0;
Timer_isOn=0; //
SecondsRealTimePv_ReConnect_T1=0;
} // end while increment
} // end first while
EEPROM.write(0,hours_lcd_1); // save hours 1 timer tp eeprom
EEPROM.write(1,minutes_lcd_1); // save minutes 1 timer tp eeprom
}
//--------------------------------Set Timer 1 Off ------------------------------
void SetTimerOff_1()
{
lcd.clear();
delay(500);
while (digitalRead(Set)==1 )
{
sprintf(txt,"[2] H:%02d-M:%02d",hours_lcd_2,minutes_lcd_2);
lcd.setCursor(0,0);
lcd.print(txt);
if (digitalRead(Exit)==1 )
{
break;     //break out of the while loop
}

//-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement)==1)
{
if (digitalRead(Increment)==1 )
{
delay(200);
minutes_lcd_2++;
}
if (digitalRead(Decrement)==1)
{
delay(200);
minutes_lcd_2--;
}
//-> perfect
if (minutes_lcd_2>59)    minutes_lcd_2=0;
if (minutes_lcd_2<0) minutes_lcd_2=0;
SecondsRealTimePv_ReConnect_T1=0;
Timer_isOn=0;
} // end while increment and decrement
} // end first while
//******************************************************************************
delay(500);     //read time for state
while (digitalRead(Set)==1 )
{
sprintf(txt,"[2] H:%02d-M:%02d",hours_lcd_2,minutes_lcd_2);
lcd.setCursor(0,0);
lcd.print(txt);
if (digitalRead(Exit)==1 )
{
break;     //break out of the while loop
}
 //-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement) == 1 )
{
if (digitalRead(Increment) == 1 )
{
delay(200);
hours_lcd_2++;
}
if (digitalRead(Decrement) == 1 )
{
delay(200);
hours_lcd_2--;
}

if  (hours_lcd_2>23) hours_lcd_2=0;
if  (hours_lcd_2<0) hours_lcd_2=0;
Timer_isOn=0; //
SecondsRealTimePv_ReConnect_T1=0;
} // end while increment
} // end first while
EEPROM.write(2,hours_lcd_2); // save hours off  timer_1 to eeprom
EEPROM.write(3,minutes_lcd_2); // save minutes off timer_1 to eeprom
}
//---------------------------------Set Timer 2--------------------------------
void SetTimerOn_2()
{
lcd.clear();
delay(500);
while (digitalRead(Set)==1 )
{
sprintf(txt,"[3] H:%02d-M:%02d",hours_lcd_timer2_start,minutes_lcd_timer2_start);
lcd.setCursor(0,0);
lcd.print(txt);
if (digitalRead(Exit)==1 )
{
break;     //break out of the while loop
}

//-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement)==1)
{
if (digitalRead(Increment)==1 )
{
delay(200);
minutes_lcd_timer2_start++;
}
if (digitalRead(Decrement)==1)
{
delay(200);
minutes_lcd_timer2_start--;
}
//-> perfect
if (minutes_lcd_timer2_start>59)    minutes_lcd_timer2_start=0;
if (minutes_lcd_timer2_start<0) minutes_lcd_timer2_start=0;
Timer_2_isOn=0; //
SecondsRealTimePv_ReConnect_T2=0;
} // end while increment and decrement
} // end first while
//******************************************************************************
delay(500);     //read time for state
while (digitalRead(Set)==1 )
{
sprintf(txt,"[3] H:%02d-M:%02d",hours_lcd_timer2_start,minutes_lcd_timer2_start);
lcd.setCursor(0,0);
lcd.print(txt);
if (digitalRead(Exit)==1 )
{
break;     //break out of the while loop
}
 //-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement) == 1 )
{
if (digitalRead(Increment) == 1 )
{
delay(200);
hours_lcd_timer2_start++;
}
if (digitalRead(Decrement) == 1 )
{
delay(200);
hours_lcd_timer2_start--;
}

if  (hours_lcd_timer2_start>23) hours_lcd_timer2_start=0;
if  (hours_lcd_timer2_start<0) hours_lcd_timer2_start=0;
Timer_2_isOn=0; //
SecondsRealTimePv_ReConnect_T2=0;
} // end while increment
} // end first while
EEPROM.write(4,hours_lcd_timer2_start); // save hours 1 timer tp eeprom
EEPROM.write(5,minutes_lcd_timer2_start); // save minutes 1 timer tp eeprom
}
//-----------------------------Set Timer 2 off-----------------------------
void SetTimerOff_2()
{
lcd.clear();
delay(500);
while (digitalRead(Set)==1 )
{
sprintf(txt,"[4] H:%02d-M:%02d",hours_lcd_timer2_stop,minutes_lcd_timer2_stop);
lcd.setCursor(0,0);
lcd.print(txt);
if (digitalRead(Exit)==1 )
{
break;     //break out of the while loop
}

//-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement)==1)
{
if (digitalRead(Increment)==1 )
{
delay(200);
minutes_lcd_timer2_stop++;
}
if (digitalRead(Decrement)==1)
{
delay(200);
minutes_lcd_timer2_stop--;
}
//-> perfect
if (minutes_lcd_timer2_stop>59)    minutes_lcd_timer2_stop=0;
if (minutes_lcd_timer2_stop<0) minutes_lcd_timer2_stop=0;
Timer_2_isOn=0; //
SecondsRealTimePv_ReConnect_T2=0;
} // end while increment and decrement
} // end first while
//******************************************************************************
delay(500);     //read time for state
while (digitalRead(Set)==1 )
{
sprintf(txt,"[4] H:%02d-M:%02d",hours_lcd_timer2_stop,minutes_lcd_timer2_stop);
lcd.setCursor(0,0);
lcd.print(txt);
if (digitalRead(Exit)==1 )
{
break;     //break out of the while loop
}
 //-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement) == 1 )
{
if (digitalRead(Increment) == 1 )
{
delay(200);
hours_lcd_timer2_stop++;
}
if (digitalRead(Decrement) == 1 )
{
delay(200);
hours_lcd_timer2_stop--;
}

if  (hours_lcd_timer2_stop>23) hours_lcd_timer2_stop=0;
if  (hours_lcd_timer2_stop<0) hours_lcd_timer2_stop=0;
Timer_2_isOn=0; //
SecondsRealTimePv_ReConnect_T2=0;
} // end while increment
} // end first while
EEPROM.write(6,hours_lcd_timer2_stop); // save hours off  timer_1 to eeprom
EEPROM.write(7,minutes_lcd_timer2_stop); // save minutes off timer_1 to eeprom
}
//--------------------------Set Battery Voltage---------------------------------
void SetLowBatteryVoltage()
{
lcd.clear();
delay(500);
while (digitalRead(Set)==1 )
{
lcd.setCursor(0,0);
lcd.print("[5] T1");
dtostrf(Mini_Battery_Voltage,4,1,txt);
lcd.setCursor(7,0);
lcd.print(txt);
lcd.setCursor(12,0);
lcd.print("V");
if (digitalRead(Exit)==1 )
{
break;     //break out of the while loop
}

//-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement)==1)
{
if (digitalRead(Increment)==1 )
{
delay(200);
Mini_Battery_Voltage+=0.1;
}
if (digitalRead(Decrement)==1)
{
delay(200);
Mini_Battery_Voltage-=0.1;
}
//-> perfect
if (Mini_Battery_Voltage>65)    Mini_Battery_Voltage=0;
if (Mini_Battery_Voltage<0) Mini_Battery_Voltage=0;
} // end while increment and decrement
}
//- save to eeporm
EEPROM.put(8, Mini_Battery_Voltage);
//-------------------------------------T2-----------------------------------------
delay(500);
while (digitalRead(Set)==1 )
{
lcd.setCursor(0,0);
lcd.print("[5] T2");
dtostrf(Mini_Battery_Voltage_T2,4,1,txt);
lcd.setCursor(7,0);
lcd.print(txt);
lcd.setCursor(12,0);
lcd.print("V");
if (digitalRead(Exit)==1 )
{
break;     //break out of the while loop
}

//-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement)==1)
{
if (digitalRead(Increment)==1 )
{
delay(200);
Mini_Battery_Voltage_T2+=0.1;
}
if (digitalRead(Decrement)==1)
{
delay(200);
Mini_Battery_Voltage_T2-=0.1;
}
//-> perfect
if (Mini_Battery_Voltage_T2>65)    Mini_Battery_Voltage_T2=0;
if (Mini_Battery_Voltage_T2<0) Mini_Battery_Voltage_T2=0;
} // end while increment and decrement
}
//-> save to eepprom
EEPROM.put(12,Mini_Battery_Voltage_T2);
} //- end set low voltage
//-----------------------------------Set Start Voltage----------------------------------------------
void SetStartUpLoadsVoltage()
{
lcd.clear();
delay(500);
while (digitalRead(Set)==1 )
{
lcd.setCursor(0,0);
lcd.print("[6] T1");
dtostrf(StartLoadsVoltage,4,1,txt);
lcd.setCursor(7,0);
lcd.print(txt);
lcd.setCursor(12,0);
lcd.print("V");
if (digitalRead(Exit)==1 )
{
break;     //break out of the while loop
}
//-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement)==1)
{
if (digitalRead(Increment)==1 )
{
delay(200);
StartLoadsVoltage+=0.1;
}
if (digitalRead(Decrement)==1)
{
delay(200);
StartLoadsVoltage-=0.1;
}
//-> perfect
if (StartLoadsVoltage>65)    StartLoadsVoltage=0;
if (StartLoadsVoltage<0)     StartLoadsVoltage=0;
} // end while increment and decrement
}
//- save to eeporm
EEPROM.put(16,StartLoadsVoltage);
//-------------------------------------T2-----------------------------------------
delay(500);
while (digitalRead(Set)==1 )
{
lcd.setCursor(0,0);
lcd.print("[6] T2");
dtostrf(StartLoadsVoltage_T2,4,1,txt);
lcd.setCursor(7,0);
lcd.print(txt);
lcd.setCursor(12,0);
lcd.print("V");
if (digitalRead(Exit)==1 )
{
break;     //break out of the while loop
}
//-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement)==1)
{
if (digitalRead(Increment)==1 )
{
delay(200);
StartLoadsVoltage_T2+=0.1;
}
if (digitalRead(Decrement)==1)
{
delay(200);
StartLoadsVoltage_T2-=0.1;
}
//-> perfect
if (StartLoadsVoltage_T2>65)    StartLoadsVoltage_T2=0;
if (StartLoadsVoltage_T2<0)     StartLoadsVoltage_T2=0;
} // end while increment and decrement
}
//-> save to eepprom
EEPROM.put(20,StartLoadsVoltage_T2);
} // end startupvoltage
//------------------------------------ Startup Timer 1 -----------------------------------------
void Startup_Timers()
{
lcd.clear();
delay(500);
while (digitalRead(Set)==1 )
{
sprintf((char*)txt,"[7] T1  %03d  S",startupTIme_1);
lcd.setCursor(0,0);
lcd.print(txt);
if (digitalRead(Exit)==1 )
{
break;     //break out of the while loop
}

//-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement)==1)
{
if (digitalRead(Increment)==1 )
{
delay(50);
startupTIme_1++;
}
if (digitalRead(Decrement)==1)
{
delay(50);
startupTIme_1--;
}
//-> perfect
if (startupTIme_1>900)    startupTIme_1=0;
if (startupTIme_1<0) startupTIme_1=0;
} // end while increment and decrement
} // end first while
EEPROM.put(24,startupTIme_1);
//******************************************************************************
delay(500);     //read time for state
while (digitalRead(Set)==1 )
{
sprintf((char*)txt,"[7] T2  %03d  S",startupTIme_2);
lcd.setCursor(0,0);
lcd.print(txt);
if (digitalRead(Exit)==1 )
{
break;     //break out of the while loop
}
 //-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement) == 1 )
{
if (digitalRead(Increment) == 1 )
{
delay(50);
startupTIme_2++;
}
if (digitalRead(Decrement) == 1 )
{
delay(50);
startupTIme_2--;
}

if  (startupTIme_2>900) startupTIme_2=0;
if  (startupTIme_2<0) startupTIme_2=0;
} // end while increment
} // end first while
EEPROM.put(26,startupTIme_2);
} // end startup timer
//------------------------------------Set Time--------------------------------------------------
void SetDS1307_Time()
{
DateTime now = rtc.now();
lcd.clear();
delay(500);
set_ds1307_hours=now.hour();
set_ds1307_minutes=now.minute();
set_ds1307_day=now.day();
set_ds1307_month=now.month();
set_ds1307_year=now.year();
while (digitalRead(Set)==1 )
{
sprintf((char*)txt,"[8] H:%02d-M:%02d",set_ds1307_hours,set_ds1307_minutes);
lcd.setCursor(0,0);
lcd.print(txt);
if (digitalRead(Exit)==1 )
{
break;     //break out of the while loop
}
//-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement)==1)
{
if (digitalRead(Increment)==1 )
{
delay(200);
set_ds1307_hours++;
}
if (digitalRead(Decrement)==1)
{
delay(200);
set_ds1307_hours--;
}
//-> perfect
if (set_ds1307_hours>23)    set_ds1307_hours=0;
if (set_ds1307_hours<0) set_ds1307_hours=0;
} // end while increment and decrement
} // end first while
//-----------------------------------------Set Minutes------------------------------------------
delay(500); 
while (digitalRead(Set)==1 )
{
sprintf(txt,"[8] H:%02d-M:%02d",set_ds1307_hours,set_ds1307_minutes);
lcd.setCursor(0,0);
lcd.print(txt);
if (digitalRead(Exit)==1 )
{
break;     //break out of the while loop
}
//-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement)==1)
{
if (digitalRead(Increment)==1 )
{
delay(200);
set_ds1307_minutes++;
}
if (digitalRead(Decrement)==1)
{
delay(200);
set_ds1307_minutes--;
}
//-> perfect
if (set_ds1307_minutes>59)    set_ds1307_minutes=0;
if (set_ds1307_minutes<0)     set_ds1307_minutes=0;
} // end while increment and decrement
} // end first while
rtc.adjust(DateTime(set_ds1307_year, set_ds1307_month, set_ds1307_day, set_ds1307_hours, set_ds1307_minutes, 0));
//-----------------------------------------Set Date-------------------------------------------
delay(500);
while (digitalRead(Set)==1 )
{

sprintf(txt,"[9] %02d/%02d/%02d",set_ds1307_day,set_ds1307_month,set_ds1307_year);
lcd.setCursor(0,0);
lcd.print(txt);
if (digitalRead(Exit)==1 )
{
break;     //break out of the while loop
}
//-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement)==1)
{
if (digitalRead(Increment)==1 )
{
delay(100);
set_ds1307_day++;
}
if (digitalRead(Decrement)==1)
{
delay(100);
set_ds1307_day--;
}
//-> perfect
if (set_ds1307_day>31)    set_ds1307_day=0;
if (set_ds1307_day<0)     set_ds1307_day=0;
} // end while increment and decrement
} // end first while
//----------------------------------------Set Month---------------------------------
delay(500);
while (digitalRead(Set)==1 )
{
sprintf((char*)txt,"[9] %02d/%02d/%02d",set_ds1307_day,set_ds1307_month,set_ds1307_year);
lcd.setCursor(0,0);
lcd.print(txt);
if (digitalRead(Exit)==1 )
{
break;     //break out of the while loop
}
//-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement)==1)
{
if (digitalRead(Increment)==1 )
{
delay(100);
set_ds1307_month++;
}
if (digitalRead(Decrement)==1)
{
delay(100);
set_ds1307_month--;
}
//-> perfect
if (set_ds1307_month>12)    set_ds1307_month=0;
if (set_ds1307_month<0)     set_ds1307_month=0;
} // end while increment and decrement

} // end first while
//------------------------------------Set year--------------------------------------------
delay(500);
while (digitalRead(Set)==1 )
{
sprintf((char*)txt,"[9] %02d/%02d/%02d",set_ds1307_day,set_ds1307_month,set_ds1307_year);
lcd.setCursor(0,0);
lcd.print(txt);
if (digitalRead(Exit)==1 )
{
break;     //break out of the while loop
}
//-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement)==1)
{
if (digitalRead(Increment)==1 )
{
delay(100);
set_ds1307_year++;
}
if (digitalRead(Decrement)==1)
{
delay(100);
set_ds1307_year--;
}
//-> perfect
if (set_ds1307_year>2030)    set_ds1307_year=0;
if (set_ds1307_year<0)     set_ds1307_year=0;
} // end while increment and decrement
} // end first while
rtc.adjust(DateTime(set_ds1307_year, set_ds1307_month, set_ds1307_day, set_ds1307_hours, set_ds1307_minutes, 0));
} // end setDS1307
 //-------------------------------CHECK TIMER IN RANGE--------------------------------------------
 //-------------------Check for timer activation inside range--------------------
 void CheckForTimerActivationInRange()
 {
 if (RunOnBatteryVoltageMode==0)
 {
 //-> first compare is hours
 if(ReadHours() > hours_lcd_1 && ReadHours()< hours_lcd_2)
 {
 Timer_isOn=1;

 }
 //-> seconds compare hours if equal now then compare minutes
 if(ReadHours()== hours_lcd_1 || ReadHours()== hours_lcd_2)
 {
 if(ReadHours()==hours_lcd_1)
 {
 //-> minutes must be bigger
 if(ReadMinutes()>=minutes_lcd_1) Timer_isOn=1;
 }
 if(ReadHours()==hours_lcd_2)
 {
 //-> minutes must be less
 if(ReadMinutes()< minutes_lcd_2) Timer_isOn=1;
 }
 }
 //------------------------------Timer 2-----------------------------------------
 if(ReadHours() > hours_lcd_timer2_start && ReadHours()< hours_lcd_timer2_stop)
 {
 Timer_2_isOn=1;
 }
 //-> seconds compare hours if equal now then compare minutes
 if(ReadHours()== hours_lcd_timer2_start || ReadHours()== hours_lcd_timer2_stop )
 {
 if(ReadHours()==hours_lcd_timer2_start)
 {
 //-> minutes must be bigger
 if(ReadMinutes()>=minutes_lcd_timer2_start) Timer_2_isOn=1;
 }
 if(ReadHours()==hours_lcd_timer2_stop)
 {
 //-> minutes must be less
 if(ReadMinutes()<minutes_lcd_timer2_stop) Timer_2_isOn=1;
 }
 }
 } // run on battery voltage mode
 }  // end function
 //**************************************1****************************************
 void CheckForTimerActivationOutRange()
 {
 if (RunOnBatteryVoltageMode==0)
 {
 //------------------------------First Timer-------------------------------------
 if (ReadHours() < hours_lcd_1  && ReadHours() < hours_lcd_2 )
 {
 Timer_isOn=0;
 }

 if (ReadHours() > hours_lcd_1  && ReadHours() > hours_lcd_2 )
 {
 Timer_isOn=0;
 }


 if (ReadHours()==hours_lcd_1)
 {
 if(ReadMinutes() < minutes_lcd_1)
 {
 Timer_isOn=0;
 }
 }
 //-> check for hours
 if (ReadHours()==hours_lcd_2)
 {
 if(ReadMinutes() > minutes_lcd_2)
 {
 Timer_isOn=0;
 }
 }
 //----------------------------Second Timer--------------------------------------
 if (ReadHours() < hours_lcd_timer2_start  && ReadHours() < hours_lcd_timer2_stop )
 {
 Timer_2_isOn=0;
 }

 if (ReadHours() > hours_lcd_timer2_start  && ReadHours() > hours_lcd_timer2_stop )
 {
 Timer_2_isOn=0;
 }


 if (ReadHours()==hours_lcd_timer2_start)
 {
 if(ReadMinutes() < minutes_lcd_timer2_start)
 {
 Timer_2_isOn=0;
 }
 }
 //-> check for hours
 if (ReadHours()==hours_lcd_timer2_stop)
 {
 if(ReadMinutes() > minutes_lcd_timer2_stop)
 {
 Timer_2_isOn=0;
 }
 }
 //--------------------------------End of Second Timer
 } // end of run on battery voltage
 }
 //-----------------------------------Read Time-------------------------------------------------
 //-------------------------------Read Seconds-----------------------------------
 unsigned short ReadSeconds()
 {
  DateTime now = rtc.now();
 Full_Seconds=now.second();
 return Full_Seconds;
 }

 //-------------------------------Read Minutes-----------------------------------
 unsigned short ReadMinutes()
 {
  DateTime now = rtc.now();
 	Full_Seconds=now.minute();
 	return Full_Seconds;
 }
 //----------------------------Read hours----------------------------------------
 unsigned short ReadHours()
 {
  DateTime now = rtc.now();

 	Full_Seconds=now.hour();
 	return Full_Seconds;
 }

 //----------------------------EEPROM Load------------------------------------------------
 void EEPROM_Load()
 {
//*****************timer 1****************
hours_lcd_1=EEPROM.read(0);
minutes_lcd_1=EEPROM.read(1);
hours_lcd_2=EEPROM.read(2);
minutes_lcd_2=EEPROM.read(3);
//*****************timer 2*****************
hours_lcd_timer2_start=EEPROM.read(4);
minutes_lcd_timer2_start=EEPROM.read(5);
hours_lcd_timer2_stop=EEPROM.read(6);
minutes_lcd_timer2_stop=EEPROM.read(7);
//**********************************************
ByPassState=0;   // enable is zero  // delete function to be programmed for rom spac
Timer_Enable=1;      // delete function to be programmed for rom space
RunOnBatteryVoltageMode=EEPROM.read(28);
UPSMode=EEPROM.read(29) ;   // ups mode
EEPROM.get(8,Mini_Battery_Voltage);
EEPROM.get(16,StartLoadsVoltage);
EEPROM.get(24,startupTIme_1);
EEPROM.get(26,startupTIme_2);
EEPROM.get(12,Mini_Battery_Voltage_T2);
EEPROM.get(20,StartLoadsVoltage_T2);
}
//------------------------------------------RunTimersCheckNow---------------------------------------
void RunTimersNowCheck()
{
if(digitalRead(Exit)==1 && digitalRead(Increment)==0 && digitalRead(Decrement)==0 && digitalRead(Set)==1)
{
//Backlight=1;
LCD_ReConfig();
//UpdateScreenTime=0; // if user pressed the button zero counter of dipslay backlight
}
//-----------------------------Bypass Mode -------------------------------------
if(digitalRead(Increment)==1 && digitalRead(Exit)==0)
{
//Backlight=1;
LCD_ReConfig();
//UpdateScreenTime=0; // if user pressed the button zero counter of dipslay backlight
delay(2500);
if (digitalRead(Increment)==1 && digitalRead(Exit)==0)
{
delay(2500);
if (digitalRead(Increment)==1 && digitalRead(Exit)==0)
{
RunLoadsByBass++;
if (  RunLoadsByBass==1 ) digitalWrite(Relay_L_Solar,1);
if (RunLoadsByBass>=2 )
{
digitalWrite(Relay_L_Solar_2,1);
}
}
}
}
//---------------------------------Reset to Summer time-------------------------
if (digitalRead(Increment)==1 && digitalRead(Exit)==1 && digitalRead(Decrement)==0)      // first
{
//Backlight=1;
LCD_ReConfig();
//UpdateScreenTime=0; // if user pressed the button zero counter of dipslay backlight
delay(1000);
if ( digitalRead(Increment)==1 && digitalRead(Exit)==1 && digitalRead(Decrement)==0)
{
delay(1000);
EEPROM_FactorySettings(1);        // summer time
delay(100);
EEPROM_Load();    // read the new values from epprom
lcd.setCursor(0,1);
lcd.print("Reset Factory");
delay(1000);
lcd.clear();
}
}
//-----------------RunOnBatteryVoltageMode--------------------------------------
if (digitalRead(Increment)==0 && digitalRead(Exit)==1 && digitalRead(Decrement)==1)      // first
{
LCD_ReConfig();
delay(2000);
if ( digitalRead(Increment)==0 && digitalRead(Exit)==1 && digitalRead(Decrement)==1)
{
//-> activate run on battery voltage
//RunOnBatteryVoltageMode=~RunOnBatteryVoltageMode;
if(RunOnBatteryVoltageMode == 0 ) RunOnBatteryVoltageMode=1 ; else if (RunOnBatteryVoltageMode==1 ) RunOnBatteryVoltageMode=0;
if (RunOnBatteryVoltageMode==0 )
{
  lcd.setCursor(0,1);
  lcd.print("Timer Mode");
}
 
if (RunOnBatteryVoltageMode==1) 
{
  lcd.setCursor(0,1);
  lcd.print("Voltage Mode");
  } 
EEPROM.write(28,RunOnBatteryVoltageMode);
delay(1000);
lcd.clear();
}
}

if(digitalRead(Decrement)==1 && digitalRead(Exit)==0)
{
LCD_ReConfig();
//Backlight=1;
//UpdateScreenTime=0; // if user pressed the button zero counter of dipslay backlight
delay(2500);
if (digitalRead(Decrement)==1 && digitalRead(Exit)==0)
{
delay(2500);
if (digitalRead(Decrement)==1 && digitalRead(Exit)==0)
{
TurnOffLoadsByPass=1;
RunLoadsByBass=0;
digitalWrite(Relay_L_Solar,0);
digitalWrite(Relay_L_Solar_2,0);
lcd.setCursor(15,0);
lcd.print(" ");

}
}
}

//------------------------------UPS Mode --------------------------------
if (digitalRead(Set)==0 && digitalRead(Decrement)==0 && digitalRead(Increment)==0  && digitalRead(Exit)==1)
{
LCD_ReConfig();
//Backlight=1;
delay(2000);
if (digitalRead(Set)==0 && digitalRead(Decrement)==0 && digitalRead(Increment)==0  && digitalRead(Exit)==1)
{
if (UPSMode==0)
{
UPSMode=1;
EEPROM.write(29,UPSMode);
lcd.setCursor(0,1);
lcd.print("UPS ON");
delay(1000);
}
else
{
UPSMode=0;
EEPROM.write(29,UPSMode);
lcd.setCursor(0,1);
lcd.print("UPS OFF");
delay(1000);
}
lcd.clear();
} // end
} // end of button
}  // end function
//-----------------------------------------EEPROM Factory Settings----------------------------
void EEPROM_FactorySettings(char period)
{
if(period==1) // summer  timer
{
if(SystemBatteryMode==12)
{
Mini_Battery_Voltage=12.0;
StartLoadsVoltage=13.0;
Mini_Battery_Voltage_T2=12.3,
StartLoadsVoltage_T2=13.2;
}
if(SystemBatteryMode==24)
{
Mini_Battery_Voltage=24.5;
StartLoadsVoltage=25.5;
Mini_Battery_Voltage_T2=25.0,
StartLoadsVoltage_T2=26.0;
}
if(SystemBatteryMode==48)
{
Mini_Battery_Voltage=49.0;
StartLoadsVoltage=52.0;
Mini_Battery_Voltage_T2=50.0,
StartLoadsVoltage_T2=53.0;
}
startupTIme_1 =90;
startupTIme_2=120;
//*****************timer 1****************
EEPROM.write(0,8);  // writing start hours
EEPROM.write(1,0);    // writing  start minutes
EEPROM.write(2,17);    // writing off hours
EEPROM.write(3,0);    // writing off minutes
//****************timer 2********************
EEPROM.write(4,9);  // writing start hours
EEPROM.write(5,0);    // writing  start minutes
EEPROM.write(6,17);    // writing off hours
EEPROM.write(7,0);    // writing off minutes
EEPROM.write(28,0);    // run on battery voltage mode
EEPROM.write(29,0); // ups mode
//**********************************************
EEPROM.put(8,Mini_Battery_Voltage);
EEPROM.put(12,Mini_Battery_Voltage_T2);
EEPROM.put(16,StartLoadsVoltage);
EEPROM.put(20,StartLoadsVoltage_T2);
EEPROM.put(24,startupTIme_1);
EEPROM.put(26,startupTIme_2);

} // end if period
}
//----------------------------------------Check Battery System Mode------------------------------
void CheckSystemBatteryMode()
{
if (Vin_Battery>= 35 && Vin_Battery <= 60) SystemBatteryMode=48;
else if (Vin_Battery>=18 && Vin_Battery <=32) SystemBatteryMode=24;
else if (Vin_Battery >=1 && Vin_Battery<= 16 ) SystemBatteryMode=12;
else SystemBatteryMode=24; // take it as default
}
//----------------------------------LCD Reconfig()---------------------------------
void LCD_ReConfig()
{
digitalWrite(Backlight,1);
UpdateScreenTime=0;
}
//---------------------------------Check Time Occurred ON------------------------------------------
 char CheckTimeOccuredOn(char seconds_required, char minutes_required,char hours_required)
 {
	DateTime now = rtc.now();

	if (now.hour()==hours_required && now.minute()==minutes_required)
	{
	return 1;
	}
	else {
		return 0;
	}
 }
 //---------------------------------Check Time Occured OFF----------------------------------------
 char CheckTimeOccuredOff(char seconds_required, char minutes_required,char hours_required)
 {
	DateTime now = rtc.now();

	if (now.hour()==hours_required && now.minute()==minutes_required)
	{
	return 1;
	}
	else {
		return 0;
	}
 }
//---------------------------------Check Timers-------------------------------------------------
void Check_Timers()
{
if(RunOnBatteryVoltageMode==0)
{
//-> timer start
matched_timer_1_start=CheckTimeOccuredOn(seconds_lcd_1,minutes_lcd_1,hours_lcd_1);
matched_timer_1_stop=CheckTimeOccuredOff(seconds_lcd_2,minutes_lcd_2,hours_lcd_2);
matched_timer_2_start=CheckTimeOccuredOn(seconds_lcd_timer2_start,minutes_lcd_timer2_start,hours_lcd_timer2_start);
matched_timer_2_stop=CheckTimeOccuredOff(seconds_lcd_timer2_stop,minutes_lcd_timer2_stop,hours_lcd_timer2_stop);
//---------------------------- Timer 1 -----------------------------------------
//-> turn Load On
if (matched_timer_1_start==1)
{
Timer_isOn=1;
TurnOffLoadsByPass=0;

//-> when grid is available and timer is on after grid so access the condition to active timer after grid is off
if (digitalRead(AC_Available)==1 && Timer_Enable==1  && Vin_Battery >= StartLoadsVoltage && RunWithOutBattery==false )
{
digitalWrite(Relay_L_Solar,1);
}
//-> if run with out battery is selected
if (digitalRead(AC_Available)==1 && Timer_Enable==1  && RunWithOutBattery==true )
{
digitalWrite(Relay_L_Solar,1);
}
} // end if ac_available
//-> Turn Load off
//******************************************************************************
if (matched_timer_1_stop==1)
{
Timer_isOn=0;        // to continue the timer after breakout the timer when grid is available
///EEPROM_write(0x49,0);        //- save it to eeprom if power is cut
//-> when grid is available and timer is on after grid so access the condition to active timer after grid is off
if (digitalRead(AC_Available)==1 && Timer_Enable==1  &&  RunWithOutBattery==false  )
{
//for the turn off there is no need for delay
SecondsRealTimePv_ReConnect_T1=0;
CountSecondsRealTimePv_ReConnect_T1=0;
digitalWrite(Relay_L_Solar,0); // relay off

}
if (digitalRead(AC_Available)==1 && Timer_Enable==1  && RunWithOutBattery==true  )
{
//for the turn off there is no need for delay
SecondsRealTimePv_ReConnect_T1=0;
CountSecondsRealTimePv_ReConnect_T1=0;
digitalWrite(Relay_L_Solar,0); // relay off
}
}
//}// end if of ac_available
//-------------------------- Timer 1 End----------------------------------------
//------------------------- Timer 2 Start---------------------------------------
if (matched_timer_2_start==1)
{
Timer_2_isOn=1;
TurnOffLoadsByPass=0;     // this variable just for if user shutdown loads and don't want to reactivated so it will be zeroed until next timer
///EEPROM_write(0x50,1);        //- save it to eeprom if power is cut
//-> when grid is available and timer is on after grid so access the condition to active timer after grid is off
if (digitalRead(AC_Available)==1 && Timer_Enable==1  && Vin_Battery >= StartLoadsVoltage_T2 && RunWithOutBattery==false)
{
digitalWrite(Relay_L_Solar_2,1);

}

if (digitalRead(AC_Available)==1 && Timer_Enable==1  && RunWithOutBattery==true)
{
digitalWrite(Relay_L_Solar_2,1);
}

} // end if ac_available


if (matched_timer_2_stop==1)
{
Timer_2_isOn=0;        // to continue the timer after breakout the timer when grid is available
///EEPROM_write(0x50,0);        //- save it to eeprom if power is cut
//-> when grid is available and timer is on after grid so access the condition to active timer after grid is off
if (digitalRead(AC_Available)==1  && Timer_Enable==1 && RunWithOutBattery==false )
{
///SolarOnGridOff_2=0; // to enter once again in the interrupt
//for the turn off there is no need for delay
digitalWrite(Relay_L_Solar_2,0);
SecondsRealTimePv_ReConnect_T2=0;
CountSecondsRealTimePv_ReConnect_T2=0;

}

if (digitalRead(AC_Available)==1 && Timer_Enable==1  && RunWithOutBattery==true )
{
SecondsRealTimePv_ReConnect_T2=0;
CountSecondsRealTimePv_ReConnect_T2=0;
digitalWrite(Relay_L_Solar_2,0); // relay off 
}

} // end match timer stop
} //end batteryvoltagemode if

//*******************************************************************************
//-------------------------Bypass System----------------------------------------
if(digitalRead(AC_Available)==0 &&  UPSMode==0 )   // voltage protector is not enabled
{
//delay(250);     // for error to get one seconds approxmiallty
//SecondsRealTime++;
CountSecondsRealTime=1;
if(SecondsRealTime >= startupTIme_1 && digitalRead(AC_Available)==0)
{

digitalWrite(Relay_L_Solar,1);
}
if(SecondsRealTime >= startupTIme_2 && digitalRead(AC_Available)==0)
{

digitalWrite(Relay_L_Solar_2,1);
}

} // end function of voltage protector

//-------------------------Bypass Mode Upo Mode---------------------------------
 if(digitalRead(AC_Available)==0 && UPSMode==1 )   // voltage protector is not enabled
{
//delay(250);       // for error to get one seconds approxmiallty
//SecondsRealTime++;
CountSecondsRealTime=1;
if( digitalRead(AC_Available)==0 && LoadsAlreadySwitchedOFF==0)
{

LoadsAlreadySwitchedOFF=1;
digitalWrite(Relay_L_Solar,0);
digitalWrite(Relay_L_Solar_2,0);
}
if(SecondsRealTime >= startupTIme_1 && digitalRead(AC_Available)==0 && LoadsAlreadySwitchedOFF==1 )
{
digitalWrite(Relay_L_Solar,1);
}
if(SecondsRealTime >= startupTIme_2 && digitalRead(AC_Available)==0 && LoadsAlreadySwitchedOFF==1 )
{
digitalWrite(Relay_L_Solar_2,1);
}
} // end function of voltage protector
//------------------------Functions for reactiving timers------------------------
/*
 these function is used for reactiving timers when grid available in the same timer is on or off
*/
//-> if the  ac is shutdown and timer is steel in the range of being on  so reactive timer 1
if (digitalRead(AC_Available)==1 && Timer_isOn==1 && Vin_Battery >= StartLoadsVoltage && RunWithOutBattery==false && TurnOffLoadsByPass==0 && RunOnBatteryVoltageMode ==0 )
{

//SecondsRealTimePv_ReConnect_T1++;
CountSecondsRealTimePv_ReConnect_T1=1;
//delay(200);
if (  SecondsRealTimePv_ReConnect_T1 > startupTIme_1)    digitalWrite(Relay_L_Solar,1);

}
if (digitalRead(AC_Available)==1 && Timer_isOn==1  && RunWithOutBattery==true && TurnOffLoadsByPass==0 && RunOnBatteryVoltageMode ==0 )
{
//econdsRealTimePv_ReConnect_T1++;
CountSecondsRealTimePv_ReConnect_T1=1;
//delay(200);

if (  SecondsRealTimePv_ReConnect_T1 > startupTIme_1) digitalWrite(Relay_L_Solar,1);

}
//-> if the  ac is shutdown and timer is steel in the range of being on  so reactive timer 2
if (digitalRead(AC_Available)==1 && Timer_2_isOn==1 && Vin_Battery >= StartLoadsVoltage_T2 && RunWithOutBattery==false && TurnOffLoadsByPass==0 && RunOnBatteryVoltageMode ==0)     //run with battery
{
//SecondsRealTimePv_ReConnect_T2++;
CountSecondsRealTimePv_ReConnect_T2=1;
//delay(50);
if (  SecondsRealTimePv_ReConnect_T2 > startupTIme_2)
 digitalWrite(Relay_L_Solar_2,1);
}

if ( digitalRead(AC_Available)==1 && Timer_2_isOn==1 &&  RunWithOutBattery==true && TurnOffLoadsByPass==0 && RunOnBatteryVoltageMode ==0)            //run without battery
{
//SecondsRealTimePv_ReConnect_T2++;
CountSecondsRealTimePv_ReConnect_T2=1;
//delay(50);
if (  SecondsRealTimePv_ReConnect_T2 > startupTIme_2)
 digitalWrite(Relay_L_Solar_2,1);
}
//-------------------------------RunOnBatteryMode-------------------------------
 if ( digitalRead(AC_Available)==1 && Vin_Battery >= StartLoadsVoltage && RunWithOutBattery==false && TurnOffLoadsByPass==0 && RunOnBatteryVoltageMode ==1 )
{

//SecondsRealTimePv_ReConnect_T1++;
CountSecondsRealTimePv_ReConnect_T1=1;
//delay(200);
if (  SecondsRealTimePv_ReConnect_T1 > startupTIme_1)     digitalWrite(Relay_L_Solar,1);
}

if ( digitalRead(AC_Available)==1 && Vin_Battery >= StartLoadsVoltage_T2 && RunWithOutBattery==false && TurnOffLoadsByPass==0 && RunOnBatteryVoltageMode ==1)     //run with battery
{
//SecondsRealTimePv_ReConnect_T2++;
CountSecondsRealTimePv_ReConnect_T2=1;
//delay(50);
if (  SecondsRealTimePv_ReConnect_T2 > startupTIme_2) digitalWrite(Relay_L_Solar_2,1);
}
//------------------------------Turn Off Loads----------------------------------
//--Turn Load off when battery Voltage  is Low and AC Not available and Bypass is enabled
if (Vin_Battery<Mini_Battery_Voltage &&  digitalRead(AC_Available)==1  && RunWithOutBattery==false )
{
CountSecondsRealTimePv_ReConnect_T1=0;
SecondsRealTimePv_ReConnect_T1=0;
CountCutSecondsRealTime_T1=1;
Start_Timer_0_A();         // give some time for battery voltage
}

//--Turn Load off when battery Voltage  is Low and AC Not available and Bypass is enabled
if (Vin_Battery<Mini_Battery_Voltage_T2 &&  digitalRead(AC_Available)==1  &&  RunWithOutBattery==false )
{
CountSecondsRealTimePv_ReConnect_T2=0;
SecondsRealTimePv_ReConnect_T2=0;
CountCutSecondsRealTime_T2=1;
Start_Timer_0_A();         // give some time for battery voltage
}
}// end of check timers
//------------------------Auto program For battery------------------------------
//@this program used for running timers without battery and to be set auto
void AutoRunWithOutBatteryProtection()
{
if (Vin_Battery==0)
{
RunWithOutBattery=true;
}
else
{
RunWithOutBattery=false;
}
}
//----------------------------------------Start Timer-+-----------------------------------------
void Start_Timer_0_A()
{
Read_Battery();
 //********************************Turn Off loads*******************************
if( CutSecondsRealTime_T1>= 15 &&  Vin_Battery<Mini_Battery_Voltage && digitalRead(AC_Available)==1 && RunLoadsByBass==0 )
{
CutSecondsRealTime_T1=0;
CountCutSecondsRealTime_T1=0;
digitalWrite(Relay_L_Solar,0);
}

if( CutSecondsRealTime_T2>= 30 && Vin_Battery<Mini_Battery_Voltage_T2 && digitalRead(AC_Available)==1  && RunLoadsByBass==0)
{
CutSecondsRealTime_T2=0;
CountCutSecondsRealTime_T2=0;
digitalWrite(Relay_L_Solar_2,0);
}

} //end start timer
//-----------------------------------------Turn Off Loads Grid----------------------------------
void TurnLoadsOffWhenGridOff()
{

if( digitalRead(AC_Available)==1 && Timer_isOn==0 && RunLoadsByBass==0  && RunOnBatteryVoltageMode==0)
{
SecondsRealTime=0;
CountSecondsRealTime=0;
SecondsRealTimePv_ReConnect_T1=0;
CountSecondsRealTimePv_ReConnect_T1=0;
digitalWrite(Relay_L_Solar,0);

}

if (digitalRead(AC_Available)==1 && Timer_2_isOn==0 && RunLoadsByBass==0 && RunOnBatteryVoltageMode==0)  // it must be   Timer_2_isOn==0    but because of error in loading eeprom value
{
SecondsRealTime=0;
CountSecondsRealTime=0;
SecondsRealTimePv_ReConnect_T2=0;
CountSecondsRealTimePv_ReConnect_T2=0;
digitalWrite(Relay_L_Solar_2,0);

}

//-> upo mode
if (digitalRead(AC_Available)==1 &&  RunLoadsByBass==0 && UPSMode==1 && LoadsAlreadySwitchedOFF==1)
{
LoadsAlreadySwitchedOFF=0;
SecondsRealTime=0;
SecondsRealTimePv_ReConnect_T1=0;
SecondsRealTimePv_ReConnect_T2=0;
CountSecondsRealTime=0;
CountSecondsRealTimePv_ReConnect_T1=0;
CountSecondsRealTimePv_ReConnect_T2=0;
digitalWrite(Relay_L_Solar_2,0);
digitalWrite(Relay_L_Solar,0);
}
}

//----------------------------------------Timer for counting seconds-------------------------------
void Timer_Seconds()
{
noInterrupts();
TCCR1A = 0; // very important 
TCCR1B = 0; // very important 
OCR1A=7800;  // 1 second 
TCCR1B |= (1<< CS10) | (1<<CS12) | (1<WGM12); // 1024 prescalar 
TIMSK1 |= (1 << OCIE1A) ;  // enabling interrupts 
interrupts();
}
//--------------------------------------Timer Interrupt----------------------------------------
ISR(TIMER1_COMPA_vect) 
{
TCNT1=0;   // very important 
UpdateScreenTime++;
if (CountSecondsRealTime==1) SecondsRealTime++;                                     // for counting real time for  grid count
if (CountSecondsRealTimePv_ReConnect_T1==1) SecondsRealTimePv_ReConnect_T1++; // for counting real time for pv connect
if(CountSecondsRealTimePv_ReConnect_T2==1) SecondsRealTimePv_ReConnect_T2++; // for counting real timer 
if(CountCutSecondsRealTime_T1==1) CutSecondsRealTime_T1++; 
if(CountCutSecondsRealTime_T2==1) CutSecondsRealTime_T2++; 
if (UpdateScreenTime==180  )  // 1800 is 60 seconds to update
{
  UpdateScreenTime=0;
  digitalWrite(Backlight,0);
  lcd.begin(16,2);
  lcd.clear();
  lcd.noCursor();
  lcd.setCursor(0,0); 
}
TurnLoadsOffWhenGridOff();
 }
 //-------------------------------------Wire Timeout----------------------------------------------
 /*
 ref: 
 - https://myhomethings.eu/en/arduino-and-the-i2c-twi-protocol/
 - https://www.fpaynter.com/2020/07/i2c-hangup-bug-cured-miracle-of-miracles-film-at-11/
 - https://www.arduino.cc/reference/en/language/functions/communication/wire/setwiretimeout/?_gl=1*115qfm6*_ga*MTAzNjA3ODQuMTY4Nzg0NzA2Mg..*_ga_NEXN8H46L5*MTY5NDMyMzk0OC42OC4xLjE2OTQzMjY3NDEuMC4wLjA.
 */
void CheckWireTimeout()
{
 if (Wire.getWireTimeoutFlag())
	{
		Wire.clearWireTimeoutFlag();   // this flag is cleared manually or cleared when  setWireTimeout() is called 
	}
}
//-----------------------------------Watch Dog timer----------------------------
void WDT_Enable()
{
//asm cli;
//asm wdr;
cli();
MCUSR &= ~(1<<WDRF);
WDTCSR |= (1<<WDCE) | (1<<WDE);     //write a logic one to the Watchdog change enable bit (WDCE) and WDE
WDTCSR |=  (1<<WDE);               //logic one must be written to WDE regardless of the previous value of the WDE bit.
//WDTCSR =  (1 <<WDP0) | (1<<WDE)  ;
sei();
}

void WDT_Prescaler_Change()
{
//asm cli;
//asm wdr;
cli();
WDTCSR |= (1<<WDCE) | (1<<WDE);
// Set new prescaler(time-out) value = 64K cycles (~0.5 s)
WDTCSR  = (1<<WDE) | (1<<WDP3) | (1<<WDP0);     // very important the equal as in datasheet examples code
//asm sei;
sei();
}

void WDT_Disable()
{
//asm cli;
//asm wdr;
cli();
MCUSR &= ~(1<<WDRF);
WDTCSR |= (1<<WDCE) | (1<<WDE);
//Turn off WDT
WDTCSR = 0x00;
//asm sei;
sei();
}
//---------------------------------------Check For Params-----------------------------------------
void CheckForParams()
{
//----------------Timer 1 ----------------
if (hours_lcd_1< 0  || hours_lcd_1 > 23)
{
hours_lcd_1=8; 
EEPROM.write(0,hours_lcd_1);  
EEPROM_Load();
}  
if (minutes_lcd_1< 0  || minutes_lcd_1 > 59)
{
minutes_lcd_1=0; 
EEPROM.write(1,minutes_lcd_1);  
EEPROM_Load();
} 
if (hours_lcd_2< 0  || hours_lcd_2 > 23)
{
hours_lcd_2=9; 
EEPROM.write(2,hours_lcd_2);  
EEPROM_Load();
}  
if (minutes_lcd_2< 0  || minutes_lcd_2 > 59)
{
minutes_lcd_2=0; 
EEPROM.write(3,minutes_lcd_2);  
EEPROM_Load();
} 
//----------------Timer 2 ------------------------
if (hours_lcd_timer2_start< 0  || hours_lcd_timer2_start > 23)
{
hours_lcd_timer2_start=8; 
EEPROM.write(4,hours_lcd_timer2_start);  
EEPROM_Load();
}  
if (minutes_lcd_timer2_start< 0  || minutes_lcd_timer2_start > 59)
{
minutes_lcd_timer2_start=0; 
EEPROM.write(5,minutes_lcd_timer2_start);  
EEPROM_Load();
} 
if (hours_lcd_timer2_stop< 0  || hours_lcd_timer2_stop > 23)
{
hours_lcd_timer2_stop=9; 
EEPROM.write(6,hours_lcd_timer2_stop);  
EEPROM_Load();
}  
if (minutes_lcd_timer2_stop< 0  || minutes_lcd_timer2_stop > 59)
{
minutes_lcd_timer2_stop=0; 
EEPROM.write(7,minutes_lcd_timer2_stop);  
EEPROM_Load();
}
//---------------------------LOW Voltage------------------------------
if (Mini_Battery_Voltage< 0  || Mini_Battery_Voltage > 65.0)
{
if (SystemBatteryMode==12) Mini_Battery_Voltage=12.0; 
if (SystemBatteryMode==24) Mini_Battery_Voltage=24.5; 
if (SystemBatteryMode==48) Mini_Battery_Voltage=49.0; 
EEPROM.put(8,Mini_Battery_Voltage);
EEPROM_Load();
}
if (Mini_Battery_Voltage_T2< 0  || Mini_Battery_Voltage_T2 > 65.0)
{
if (SystemBatteryMode==12) Mini_Battery_Voltage_T2=12.3; 
if (SystemBatteryMode==24) Mini_Battery_Voltage_T2=25.0; 
if (SystemBatteryMode==48) Mini_Battery_Voltage_T2=50.0; 
EEPROM.put(12,Mini_Battery_Voltage_T2);
EEPROM_Load();
}
//--------------------------Start Loads Voltage------------------------
if (StartLoadsVoltage< 0  || StartLoadsVoltage > 65.0)
{
if (SystemBatteryMode==12) StartLoadsVoltage=13.0; 
if (SystemBatteryMode==24) StartLoadsVoltage=25.5; 
if (SystemBatteryMode==48) StartLoadsVoltage=52.0; 
EEPROM.put(16,StartLoadsVoltage);
EEPROM_Load();
}
if (StartLoadsVoltage_T2< 0  || StartLoadsVoltage_T2 > 65.0)
{
if (SystemBatteryMode==12) StartLoadsVoltage_T2=13.2; 
if (SystemBatteryMode==24) StartLoadsVoltage_T2=26.0; 
if (SystemBatteryMode==48) StartLoadsVoltage_T2=53.0; 
EEPROM.put(20,StartLoadsVoltage_T2);
EEPROM_Load();
}
//-------------------------Startup Timers--------------------------------
if (startupTIme_1< 0  || startupTIme_1 > 900)
{
startupTIme_1=0; 
EEPROM.put(24,startupTIme_1);
EEPROM_Load();
}
if (startupTIme_2< 0  || startupTIme_2 > 900)
{
startupTIme_2=0; 
EEPROM.put(26,startupTIme_2);
EEPROM_Load();
}
//----------------------Run On Battery Voltage Mode-------------------------
if (RunOnBatteryVoltageMode < 0 || RunOnBatteryVoltageMode > 1 )
{
  RunOnBatteryVoltageMode=0;
  EEPROM.write(28,RunOnBatteryVoltageMode);
  EEPROM_Load();
}
//----------------------------UPS Mode------------------------------------
if (UPSMode < 0 || UPSMode > 1 )
{
  UPSMode=0;
  EEPROM.write(29,UPSMode);
  EEPROM_Load();
}
}
//---------------------------------------MAIN LOOP-------------------------------------------------
void setup() {
  // put your setup code here, to run once:
  Config();
  Config_Interrupts();
  EEPROM_Load();
  Timer_Seconds();
  }

void loop() {
  // put your main code here, to run repeatedly:
  CheckForParams();
  CheckForSet(); // done 
  RunTimersNowCheck(); // done 
  CheckSystemBatteryMode();  // done
  AutoRunWithOutBatteryProtection();
  CheckForTimerActivationInRange();  // done
  CheckForTimerActivationOutRange();  // done
  Screen_1();  // done 
  Check_Timers();  // done
  TurnLoadsOffWhenGridOff();  // done
  CheckWireTimeout(); 
  delay(50);
 }