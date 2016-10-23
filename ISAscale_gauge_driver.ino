/*
  This program reads CAN data coming from an ISAscale CAN shunt and presents it on a 4DSystems LCD screen. It
  will also--using a SparkFun mini-fet shield and an auxiliary PnP transistor--create pulses to drive a normal
  Tachometer to show current AMP draw and variable resistance (PWN to ground) to show SOC on a conventional
  fuel gauge. It has two auxiliary on/off outputs (again to ground) to light some indicators. One for low battery
  and the second as a warning light in case of voltage mismatch between the various segments of the battery pack.
  Our poor-man's BMS.

  It's meant to run on an EVTVDue CAN microcontroller although it should work on other arduinos with an appropriate
  CAN shield.

  Note that EVTVDue CAN microcontroller ONLY provides CAN0 and does not provide a CAN1 output.   It also ONLY supports
  the NATIVE USB port and so this code will not work on the programming port as written.
  
  copyright 2016 Rick Beebe
  portions copyright 2015 Jack Rickard, Collin Kidder
    
  This sketch uses the EVTVDue Microcontroller and is compatible with Arduino Due  Use BOARD select Arduino Due (Native USB port).

  5/24/2016 Rick Beebe
   
*/

#include <ISA.h>			// from EVTV
#include <genieArduino.h> //https://github.com/4dsystems/ViSi-Genie-Arduino-Library


//This enables syntactic sugar to allow streaming to serial port SerialUSB<<"Text";
template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; } //Sets up serial streaming Serial<<someshit;

 // Useful macros for setting and resetting bits
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))



//*********GENERAL VARIABLE   DATA ******************

ISA Sensor;  //Instantiate ISA Module Sensor object to measure current and voltage 

CAN_FRAME outframe;  //A structured variable according to due_can library for transmitting CAN data.


Genie genie;			// The structure for the 4D systems display module
#define RESETLINE 4  // Pin the display reset line is connected to. It needs a 1k-ohm resistor in-line

#define BAR_LEFT	0
#define BAR_MID	256
#define BAR_RIGHT	512

#define SOC	0
#define AMPS	1



//Other ordinary variables
float Version=1.00;
uint16_t page=300;    //EEPROM page to hold variable data.  We save time by setting up a structure and saving a large block
int i;
unsigned long elapsedtime, time228,timestamp,startime, lastime;  //Variables to compare millis for timers
boolean debug=false;
uint8_t logcycle=0;

// our defaults
float ISA_Voltage=144.0;
float ISA_Volt1=48.0;
float ISA_Volt2=48.0;
float ISA_Volt3=48.0;
float ISA_Amps=0.0;
float ISA_Used = 0.0;	//KWH used so far. Max is pack capacity
uint8_t displaySoc=0;	// percentage shown on screen


//******* END OF GENERAL VARIABLE DATA***********






//*********EEPROM  DATA ******************
/*
 * This section provides a data struction and function for reading and writing EEPROM data containing persistent configuration variable we want to retain from 
 * one session to the next.  ANy kind of legal variable can be added to the structure and periodically during main loop execution we will write all variable data as a block\
 * to the EEPROM.  This minimizes the time delay hit of EEPROM writes.  You can update these variables as often as you like in other sections of the program and 
 * it will be actually saved to EEPROM during the next block write.
 * 
 * The initialize function is normally executed on new hardware where no previously written EEPROM data is found.  You can set default values here and on the first program 
 * execution those values will be loaded and used by the program.  Subsequent changes will be written to EEPROM and during the next session, the updated variables will be 
 * retrieved from EEPROM.
 */

//Store any configuration variables or other data you want to be persistent between power cycles.  Available in myVars.variable

// increment this number to for a refresh of the EEPROM such as when changing anything in this class
#define EEPROM_CRC 23

class EEPROMvariables {
  public:
    uint8_t CANport;
    uint32_t datarate;
    uint16_t transmitime;
    uint8_t varsGood;
	 float capacity;
};
EEPROMvariables myVars;

void initializeEEPROMPage()
{
  //If no EEPROM data can be found at the EEPROM page, initialize the variables held in myVars here.  It will then be written to the designated EEPROM page
  
	myVars.CANport=0;
	myVars.datarate=500000;
	myVars.transmitime=120;
	myVars.capacity = 11.520;	// 80aH * 144 volts
	myVars.varsGood=EEPROM_CRC;
	EEPROM.write(page,myVars);

}

//********* END OF EEPROM  DATA ******************



//********************SETUP FUNCTION*******I*********
/*
 * The SETUP function in the Arduino IDE simply lists items that must be performed prior to entering the main program loop.  In this case we initialize SerialUSB 
 * communications via the USB port, set an interrupt timer for urgent outbound frames, zero our other timers, and load our EEPROM configuration data saved during the 
 * previous session.  If no EEPROM data is found, we initialize EEPROM data to default values
 * 
 */
void setup() {

	// Set up the display
	Serial.begin(38400);  // Serial0 @ 115200
	genie.Begin(Serial);   // Use Serial0 for talking to the Genie Library, and to the 4D Systems display

	// Reset the Display
	pinMode(RESETLINE, OUTPUT);  // Set D4 on Arduino to Output (4D Arduino Adaptor V2 - Display Reset)
	digitalWrite(RESETLINE, 0);  // Reset the Display via D4
	delay(100);
	digitalWrite(RESETLINE, 1);  // unReset the Display via D4
	delay (3500); //let the display start up after the reset (This is important)

	// Set the brightness/Contrast of the Display - (Not needed but illustrates how)
	// Most Displays, 1 = Display ON, 0 = Display OFF. See below for exceptions and for DIABLO16 displays.
	// For uLCD-43, uLCD-220RD, uLCD-70DT, and uLCD-35DT, use 0-15 for Brightness Control, where 0 = Display OFF, though to 15 = Max Brightness ON.
	genie.WriteContrast(1); 


	SerialUSB.begin(115200);  //Initialize our USB port which will always be redefined as SerialUSB to use the Native USB port tied directly to the SAM3X processor.

//	Timer4.attachInterrupt(sendCANframeURGENT).start(10000); // This sets an interrupt time to send an URGENT CAN frame every 10ms.  The function can be changed
		                                                      // and the time can be changed.


	lastime=startime=time228=timestamp=millis();  //Zero our other timers

	//Load/validate EEPROM variables
	Wire.begin();
	EEPROM.setWPPin(19);     
	EEPROM.read(page, myVars);
	if(myVars.varsGood != EEPROM_CRC)
		initializeEEPROMPage(); //We check to see if we have a saved page from earlier session. If not, initialize EEPROM variables and save.

	// Initialize CAN ports 
	Sensor.begin(myVars.CANport, myVars.datarate);

	//Print welcome screen and menu  
	SerialUSB<<"\n\n Startup successful. Rick Beebe -  ISAscale Gauge Driver Version "<<Version<<"\n\n";
	printMenu();    
}
   
//********************END SETUP FUNCTION*******I*********





//********************MAIN PROGRAM LOOP*******I*********
/*
 * This is the main program loop. Lacking interrupts, this loop is simply repeated endlessly.  Show is an example of code that only executes after a certain
 * amount of time has passed. myVars.transmitime.  It also uses a counter to perform a very low priority function every so many loops.
 * Any other entries will be performed with each loop and on Arduino Due, this can be very fast.
 */



void loop()
{ 
  int x, gaugeLeft, gaugeMid, gaugeRight;
  uint8_t percentCharged;
  uint8_t leftVolt, midVolt, rightVolt;


   
  if(millis()-lastime > myVars.transmitime)  //Nominally set for 120ms - do stuff on 120 ms non-interrupt clock
    {
     lastime=millis();        //Zero our timer

	  if (ISA_Used > myVars.capacity) ISA_Used = myVars.capacity;
     
	displaySoc = 100 - (uint8_t)((ISA_Used / myVars.capacity) * 100);

	leftVolt = ISA_Volt1 - ISA_Volt2 + 6;
	midVolt = ISA_Volt2 - ISA_Volt3 + 6;
	rightVolt = ISA_Volt3 - ISA_Volt1 + 6;


//     sendCANframe();
     printstatus();


	  // update the LCD
    genie.WriteObject(GENIE_OBJ_GAUGE, SOC, displaySoc);
    genie.WriteObject(GENIE_OBJ_GAUGE, AMPS, (int)ISA_Amps);
	 // And the digits
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0, (int)(ISA_Voltage * 10));
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 1, (int)(ISA_Used * 1000));

	// Bounds checking
	 gaugeLeft = (leftVolt < 0) ? 0 : leftVolt;
	 gaugeLeft = (leftVolt > 12) ? 12 : leftVolt;
	 gaugeMid = (midVolt < 0) ? 0 : midVolt;
	 gaugeMid = (midVolt > 12) ? 12 : midVolt;
	 gaugeRight = (rightVolt < 0) ? 0 : rightVolt;
	 gaugeRight = (rightVolt > 12) ? 12 : rightVolt;

    genie.WriteObject(GENIE_OBJ_SPECTRUM, 0, BAR_LEFT + gaugeLeft);
    genie.WriteObject(GENIE_OBJ_SPECTRUM, 0, BAR_MID + gaugeMid);
    genie.WriteObject(GENIE_OBJ_SPECTRUM, 0, BAR_RIGHT + gaugeRight);


      
          
     if(logcycle++ > 200) //Every 30 seconds, save configuration and data to EEPROM
       {  
        logcycle=0;
//        Timer4.stop();    //Turn off the 10ms interrupt for 228 to avoid conflicts
        EEPROM.write(page, myVars);
//        Timer4.start();
        } 
    }        
}
//********************END MAIN PROGRAM LOOP*******I*********






  
 //********************USB SERIAL INPUT FROM KEYBOARD *******I*********
 /* These routines use an automatic interrupt generated by SERIALEVENT to process keyboard input any time a string terminated with carriage return is received 
  *  from an ASCII terminal program via the USB port.  The normal program execution is interrupted and the checkforinput routine is used to examine the incoming
  *  string and act on any recognized text.
  * 
  */
   
void serialEventRun(void) 
{
   if (SerialUSB.available())checkforinput(); //If SerialUSB event interrupt is on USB port, go check for keyboard input           
}


void checkforinput()
{
  //Checks for input from SerialUSB Port 1 indicating configuration items most likely.  Handle each case.
  
   if (SerialUSB.available()) 
   {
    int inByte = SerialUSB.read();
    switch (inByte)
     {
    	 case '?':
       case 'h':
      		printMenu();
      		break; 	
     	 case 'D':
	    case 'd':
      		debug=(!debug);
      		break;
       case 'i':
       case 'I':
      		getInterval();
      		break;
	    case 'c':
       case 'C':
//      		getPort();
				getCapacity();
      		break;
       case 'K':
       case 'k':
      		getRate();
      		break;
       case 'V':
       case 'v':
      		getFloat(ISA_Voltage, "Pack Voltage");
      		break;
       case 'A':
       case 'a':
      		getAmps();
      		break;
       case 'S':
       case 's':
      		getFloat(ISA_Used, "SOC (0-capacity)");
      		break;
       case '1':
      		getFloat(ISA_Volt1, "Voltage 1 (0-12)");
      		break;
       case '2':
      		getFloat(ISA_Volt2, "Voltage 2 (0-12)");
      		break;
       case '3':
      		getFloat(ISA_Volt3, "Voltage 3 (0-12)");
      		break;
      
   	  }    
    }
}





void getInterval()
{
	SerialUSB<<"\n Enter the interval in ms between each CAN frame transmission : ";
		while(SerialUSB.available() == 0){}               
		float V = SerialUSB.parseFloat();	
		if(V>0)
		  {
        SerialUSB<<V<<"CAN frame interval\n\n";
        myVars.transmitime=abs(V);
      }
}
       
void getCapacity()
{
	SerialUSB<<"\n Enter the pack capacity in Kilowatt Hours : ";
		while(SerialUSB.available() == 0){}               
		float V = SerialUSB.parseFloat();	
		if(V>0)
		  {
        SerialUSB<<V<<" kilowatt hour capacity\n\n";
        myVars.capacity=abs(V);
      }
}

void getRate()
{
	SerialUSB<<"\n Enter the Data Rate in Kbps you want for CAN : ";
		while(SerialUSB.available() == 0){}               
		float V = SerialUSB.parseFloat();	
		if(V>0)
		  {
       SerialUSB<<V<<"\n\n";
       myVars.datarate=V*1000;
       initializeCAN(myVars.CANport);
		  }
}


void getPort()
{
	SerialUSB<<"\n Enter port selection:  c0=CAN0 c1=CAN1 ";
	while(SerialUSB.available() == 0){}               
	int P = SerialUSB.parseInt();	
	myVars.CANport=P;
	if(myVars.CANport>1) SerialUSB<<"Entry out of range, enter 0 or 1 \n";
  else 
     {
     if(myVars.CANport==0) initializeCAN(0); //If CANdo is 0, initialize CAN0 
     if(myVars.CANport==1) initializeCAN(1);    //If CANdo is 1, initialize CAN1
   	 }           
}

void getAmps() {

	SerialUSB<<"\n Enter amp draw (0-600):  ";
	while(SerialUSB.available() == 0){}
	float P = SerialUSB.parseFloat();
	if (P < 0 || P > 600)
		SerialUSB<<"Entry out of range (0-600) \n";
	else
		ISA_Amps = P;
}

void getFloat(float &var, const char *name) {

	SerialUSB<<"\n Enter "<<name<<":  ";
	while(SerialUSB.available() == 0){}               
	var = SerialUSB.parseFloat();	
}


//********************END USB SERIAL INPUT FROM KEYBOARD ****************









//******************** USB SERIAL OUTPUT TO SCREEN ****************
/*  These functions are used to send data out the USB port for display on an ASCII terminal screen.  Menus, received frames, or variable status for example
 *   
 */

void printMenu()
{
  SerialUSB<<"\f\n=========== ISAscale Gauge Driver  Version "<<Version<<" ==============\n************ List of Available Commands ************\n\n";
  SerialUSB<<"  ? or h  - Print this menu\n";
  SerialUSB<<"  c - sets CAN port ie c0 or c1\n";
  SerialUSB<<"  d - toggles debug DISPLAY FRAMES to print CAN data traffic\n";
  SerialUSB<<"  i - set interval in ms between CAN frames i550 \n";
  SerialUSB<<"  k - set data rate in kbps ie k500 \n";

  SerialUSB<<"  a -  Set AMPs (0-600)\n";
  SerialUSB<<"  s -  Set SOC% (0-100)\n";
  SerialUSB<<"  v -  Set voltage total\n";
  SerialUSB<<"  1 -  Set voltage 1\n";
  SerialUSB<<"  2 -  Set voltage 2\n";
  SerialUSB<<"  3 -  Set voltage 3\n";

 
 SerialUSB<<"**************************************************************\n==============================================================\n\n"; 
}

void printFrame(CAN_FRAME *frame,int sent)
{ 
  char buffer[300];
  sprintf(buffer,"msgID 0x%03X; %02X; %02X; %02X; %02X; %02X; %02X; %02X; %02X  %02d:%02d:%02d.%04d\n", frame->id, frame->data.bytes[0], 
  frame->data.bytes[1],frame->data.bytes[2], frame->data.bytes[3], frame->data.bytes[4], frame->data.bytes[5], frame->data.bytes[6],
  frame->data.bytes[7], hours(), minutes(), seconds(), milliseconds());
  
   if(sent)Serial<<"Sent ";
    else SerialUSB<<"Received ";       
   SerialUSB<<buffer<<"\n";
}

void printstatus()
{
  /* This function prints an ASCII statement out the SerialUSB port summarizing various data from program variables.  Typically, these variables are updated
   *  by received CAN messages that operate from interrupt routines. This routine also time stamps the moment at which it prints out.
   */
  
    char buffer[300]; 
  //  sprintf(buffer,"CAN%i Int:%ims Regen:%i%% Throt:%i%% RPM:%i MPH:%i Torq %iNm %3.1fvdc %iAmps %3.1fkW MotT1:%iC InvT5:%iC  %02d:%02d:%02d.%04d\n\n",myVars.CANport,
//    myVars.transmitime,regenpercent,throttle,rpm,mph, torq1,voltage,current,power,temperature1,temperature5,hours(),minutes(),seconds(),milliseconds());
 
//   sprintf(buffer,"CAN%i Rate:%i Int:%ims  %02d:%02d:%02d.%04d\n\n",myVars.CANport,myVars.datarate, myVars.transmitime,hours(),minutes(),seconds(),milliseconds());
   sprintf(buffer,"Voltage: %f AMPS:%f USED: %f CAPACITY: %f SOC:%f (%i)  %f %f %f\n",ISA_Voltage,ISA_Amps, ISA_Used, myVars.capacity, (ISA_Used/myVars.capacity) * 100, displaySoc, ISA_Volt1, ISA_Volt2, ISA_Volt3);
 
    SerialUSB<<buffer;
}


int milliseconds(void)
{
  int milliseconds = (int) (micros()/100) %10000 ;
  return milliseconds;
}


 int seconds(void)
{
    int seconds = (int) (micros() / 1000000) % 60 ;
    return seconds;
}


int minutes(void)
{
    int minutes = (int) ((micros() / (1000000*60)) % 60);
    return minutes;
}

    
int hours(void)
{    
    int hours   = (int) ((micros() / (1000000*60*60)) % 24);
    return hours;
}  



//******************** END USB SERIAL OUTPUT TO SCREEN ****************






//******************** CAN ROUTINES ****************************************
/* This section contains CAN routines to send and receive messages over the CAN bus
 *  INITIALIZATION routines set up CAN and are called from program SETUP to establish CAN communications.
 *  These initialization routines allow you to set filters and interrupts.  On RECEIPT of a CAN frame, an interrupt stops execution of the main program and 
 *  sends the frame to the specific routine used to process that frame by Message ID. Once processed, the main program is resumed.
 *  
 *  Frames can also be sent, either from the main control loop, or by a Timer interrupt allowing specific frames to be sent on a regular interrupt interval.
 *  
 *  For example a frame that MUST go out every 10 ms would use the Timer Interrupt in SETUP to cause a CAN function here to send a frame every 10 ms.  Other frames 
 *  could be sent normally from the main program loop or conditionally based on some received or calculated data.
 *  
 */

void initializeCAN(int which)
{
  //Initialize CAN bus 0 or 1 and set filters to capture incoming CAN frames and route to interrupt service routines in our program.
  
  if(which)  //If 1, initialize Can1.  If 0, initialize Can0.
    {
      pinMode(48,OUTPUT);
      if (Can1.begin(myVars.datarate,48)) 
        {
          SerialUSB.println("Using CAN1 - initialization completed.\n");
       //   Can1.setNumTXBoxes(3);
          Can1.setRXFilter(0, 106, 0x7FF, false);
          Can1.setCallback(0, handleCANframe);
          Can1.setRXFilter(1, 0x618, 0x7FF, false);
          Can1.setCallback(1, handleCANframe);
          Can1.setRXFilter(2, 0x274, 0x7FF, false);
          Can1.setCallback(2, handleCANframe);
          Can1.setRXFilter(3, 0x1D6, 0x7FF, false);
          Can1.setCallback(3, handleCANframe);
          Can1.setRXFilter(4, 0x30A, 0x7FF, false);
          Can1.setCallback(4, handleCANframe);
          Can1.setRXFilter(5, 0x308, 0x7ff, false);
          Can1.setCallback(5, handleCANframe);
          Can1.setRXFilter(6, 0x268, 0x700, false);
          Can1.setCallback(6, handleCANframe);           
        }
           else SerialUSB.println("CAN1 initialization (sync) ERROR\n");
    } 
  else
    {//else Initialize CAN0
     pinMode(50,OUTPUT);
     if (Can0.begin(myVars.datarate,50)) 
        {
          SerialUSB.println("Using CAN0 - initialization completed.\n");
          //Can0.setNumTXBoxes(3);
          Can0.setRXFilter(0, 0x106, 0x7FF, false);
          Can0.setCallback(0, handleCANframe);
          Can0.setRXFilter(1, 0x306, 0x7FF, false);
          Can0.setCallback(1, handleCANframe); 
          Can0.setRXFilter(2, 0x126, 0x7FF, false);
          Can0.setCallback(2, handleCANframe);
          Can0.setRXFilter(3, 0x154, 0x7FF, false);
          Can0.setCallback(3, handleCANframe);        
                 
          Can0.setRXFilter(4, 0x100, 0x700, false);
          Can0.setCallback(4, handleCANframe);
          }
        else SerialUSB.println("CAN0 initialization (sync) ERROR\n");
    }
}   

void handleCANframe(CAN_FRAME *frame)
//This routine handles CAN interrupts from a capture of any other CAN frames from the inverter not specifically targeted.  
//If you add other specific frames, do a setRXFilter and CallBack mailbox for each and do a method to catch those interrupts after the fashion
//of this one.
{  
    //This routine basically just prints the general frame received IF debug is set.  Beyond that it does nothing.
    
    if(debug) printFrame(frame,0); //If DEBUG variable is 1, print the actual message frame with a time stamp showing the time received.      
}


void handle106frame(CAN_FRAME *frame)
//This routine handles CAN interrupts from  Address 0x106 CAN frame   
{   
  float throttle;
  int rpm;
  int wheelrpm;
  int torq2;
  int torq1;
   
    throttle=round(frame->data.bytes[6]/2.56);
    rpm=word(frame->data.bytes[5],frame->data.bytes[4]);
    wheelrpm=rpm/9.73;
    torq2=(word(frame->data.bytes[3],frame->data.bytes[2]))&8191;
      if(torq2>4095)torq2=(8192-torq2)*-1;
      torq2/=10;
    torq1=(word(frame->data.bytes[1],frame->data.bytes[0]))&8191;
      if(torq1>4095)torq1=(8192-torq1)*-1;
      torq1/=10;
      
    if(debug)printFrame(frame,0); //If DEBUG variable is 1, print the actual message frame with a time stamp showing the time received.    
}


void sendCANframe()
{
        outframe.id = 0x700;            // Set our transmission address ID
        outframe.length = 8;            // Data payload 8 bytes
        outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
        outframe.rtr=1;                 //No request
        outframe.data.bytes[0]=0x00;
        outframe.data.bytes[1]=0x00;  
        outframe.data.bytes[2]=highByte(myVars.transmitime);
        outframe.data.bytes[3]=lowByte(myVars.transmitime);
        outframe.data.bytes[4]=0x00;
        outframe.data.bytes[5]=highByte(myVars.transmitime);
        outframe.data.bytes[6]=lowByte(myVars.transmitime);
        outframe.data.bytes[7]=00;
       
        if(debug) {printFrame(&outframe,1);} //If the debug variable is set, show our transmitted frame
                      
        if(myVars.CANport==0) Can0.sendFrame(outframe);    //Mail it
         else Can1.sendFrame(outframe); 


}

void sendCANframeURGENT()
{
        outframe.id = 0x700;            // Set our transmission address ID
        outframe.length = 8;            // Data payload 8 bytes
        outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
        outframe.rtr=1;                 //No request
        outframe.data.bytes[0]=0x00;
        outframe.data.bytes[1]=0x00;  
        outframe.data.bytes[2]=highByte(myVars.transmitime);
        outframe.data.bytes[3]=lowByte(myVars.transmitime);
        outframe.data.bytes[4]=0x00;
        outframe.data.bytes[5]=highByte(myVars.transmitime);
        outframe.data.bytes[6]=lowByte(myVars.transmitime);
        outframe.data.bytes[7]=00;
       
        if(debug) {printFrame(&outframe,1);} //If the debug variable is set, show our transmitted frame
                      
        if(myVars.CANport==0) Can0.sendFrame(outframe);    //Mail it
         else Can1.sendFrame(outframe); 


}

//******************** END CAN ROUTINES ****************












