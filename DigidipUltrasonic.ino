//---------------------------------------------------------------
// Code for implementing digital dipstick algorithm -------------
//---------------------------------------------------------------
#include <SD\SD.h> //SD Library
#include <Wire.h> //I2C Library
#include <SPI.h> //SPI Library
#include <RTClib\RTClib.h> //Real Time Clock Library
#include <Adafruit_MPL3115A2_Library\Adafruit_MPL3115A2.h>
//---------------------------------------------------------------
/*
Pin Configuration:
Pin #, Connects to           , Function

A0   , Sensor Signal         , Reads the sensor
A4   , RTC SDA Pin           , I2C SDA
A5   , RTC SCL Pin           , I2C SCL

2    , G through Save Button , Interrupt to Save the data (Green Button) 
3    , G through Skip Button , Interrupt to Skip one bucket (White Button)
5    , G through Select Butn , The green button on the buttom to cycle through the options
6    , G through the trigger , Starts reading the sensor (Read (trigger) Button)
8    , Display's SS          , Slave Select for the SPI communication of the display
9    , + Save LED            , Connects to the positive end of the led to show stabilized condition and write permission , G through LED Button
10   , SD shield's SS        , Slave Select for the SPI communication of the SD shield
11   , Pin 11                , MOSI (ICSP 4)
12   , Pin 12                , MISO (ICSP 1)
13   , Pin 13                , SCK  (ICSP 3)
*/
//---------------------------------------------------------------
/*
The connection between SS of the SD shield and pin 10 should be 
disconnected without disconnecting the connection between the SS and the 
resistor that it connects to.
After this, pin 10 can be used for the SS of the display. Pin 10 
is the default pin for the SS of the display. (If pin 8 is going to be used
for the SS of Display then the connection between pin 8 and RX needs to be disconnected.)

On the patch shield for J1 (The port connected to the SD and Display shields): 
1 -> SCL
2 -> SDA
3 -> 
4 -> Digital 10 (SS for SD card) (SSSD)
5 -> Digital 13
6 -> Digital 12
7 -> Digital 11
8 -> Digital 8  (SS for Display) (SSD)

*/
//---------------------------------------------------------------
// The LED on the Button 2 (save button) is connected to digital pin 9
#define greenLEDpin 9 

// The button 1 (read/trigger) is connected to digital pin 6
#define button1pin 6

// sensor wire from the water level meter is connected to the analog pin 0
//#define sensorPin A0

RTC_DS1307 RTC; // define the Real Time Clock object
//Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
// for the data logging shield, we use digital pin 10 for the SS line
const uint8_t SSSD = 10;
// for the Display shield, we use digital pin 8 for the SS line
const uint8_t SSD = 8;

const uint8_t compNum = 8; //Number of readings to compare the current reading to (to find if the reading is stabilized)

// milliseconds between reading data
//const uint8_t dataRate=10;

const int tripod = 615;

char tempString[10];  // Will be used with sprintf to create strings

volatile int bucket = 1; //variable containing the number of bucket to be measured
int previousBucket = 1; // The number of the previous bucket that was written to the disk

volatile boolean writeOK = 0; //variable defined for the interrupt
volatile uint8_t cycle=1;

int lowPercentage = 6000;
int highPercentage = 900;
int percentage = 200; //percentage times 100

uint8_t testType = 0;
uint8_t testDura = 0;

long difference;

long mean = 0L;

boolean Button1;

volatile uint8_t LWD = 2; //to switch between Length, Weight and Density
volatile boolean reading = 0;
volatile boolean firstTime = 1;

// the logging file
File logfile;

//--------------------------------------------------------------

void error()
{
	clearDisplaySPI();  // Clears display, resets cursor
	s7sSendStringSPI("SDEr");  // Displays Sd.Er. if the SD card cannot be initiallized
    setDecimalsSPI(0b001010);
    while (true)
    {

    };
}

//void error1()
//{
//    clearDisplaySPI();  // Clears display, resets cursor
//    s7sSendStringSPI("T-Er");  // Displays Sd.Er. if the SD card cannot be initiallized
//    setDecimalsSPI(0b001010);
//    delay(500);
//}
//-------------------------------------------------------------

void setup(void)
{
    Serial.begin(9600);
	delay(200);
	pinMode(11, OUTPUT);
	digitalWrite(11, LOW);
	pinMode(12, INPUT);
	pinMode(13, OUTPUT);
	digitalWrite(13, LOW);

    pinMode(7, OUTPUT);
    digitalWrite(7, HIGH);

	// -------- SPI initialization
	pinMode(SSD, OUTPUT);  // Set the SS pin as an output
	digitalWrite(SSD, HIGH);  // Set the SS pin HIGH
	SPI.begin();  // Begin SPI hardware
	SPI.setClockDivider(SPI_CLOCK_DIV128);  // Slow down SPI clock
	// --------
	Wire.begin();
	delay(200);
	// Clear the display, and then show TYCO on the display
	clearDisplaySPI();  // Clears display, resets cursor
	delay(100);
	setBrightnessSPI(255);  // High brightness
	delay(100);

	pinMode(greenLEDpin, OUTPUT);  //LED on button 9 (LED on the write button)
	pinMode(button1pin, INPUT); //  connects through the read/trigger button to G
	digitalWrite(button1pin, HIGH); //to make use of the 10k internal resistor
	pinMode(2, INPUT);
	pinMode(3, INPUT);
	digitalWrite(2, HIGH); //to make use of the 10k internal resistor for the int0
	digitalWrite(3, HIGH); //Interrupt to skip one bucket with function bNumber() for the int1
	pinMode(5, INPUT);
	digitalWrite(5, HIGH); //the button to cycle through options

	// make sure that the default SS for the SD shield is set to output
	pinMode(SSSD, OUTPUT);
	digitalWrite(SSSD, HIGH);
	
	s7sSendStringSPI("TYCO");  // Displays TYCO on all digits
	delay(200);
	clearDisplaySPI();  // Clears display, resets cursor
	delay(100);
	s7sSendStringSPI(" BY ");  
	delay(200);
	clearDisplaySPI();  // Clears display, resets cursor
	delay(100);
	s7sSendStringSPI(" AA ");  
	setDecimalsSPI(0b000110);
	delay(200);
	clearDisplaySPI();  // Clears display, resets cursor
	delay(100);
	s7sSendStringSPI(" JC "); 
	setDecimalsSPI(0b000110);
	delay(200);

	clearDisplaySPI();      



	//-------------------------------------------------------------------------------------------------------------
	// The code to decide between different options available for the test

	boolean Button5;
	boolean Button3;
	unsigned long lastTime = millis();

	//	attachInterrupt(1, cycling, FALLING); //Interrupt connected to digital pin 3 for cycling through choices

	while (testType == 0)
	{
		Button5 = digitalRead(5);
		Button3 = digitalRead(3);
		
		if (!Button3 && (millis()-lastTime)>400) {cycle++; lastTime=millis();}
		if (cycle==3){cycle=1;}
		if (!Button5){ testType=cycle; }

		switch ( cycle ) 
		{
		case 1:
			//clearDisplaySPI();
			s7sSendStringSPI("resd");
			break;
		case 2:
			//clearDisplaySPI();
			s7sSendStringSPI("conn");
			break;
		}
	}
	if(testType==1){cycle = 1;}
    else{ cycle = 4; LWD = 1; }
	delay(200);
	testType -= 1;
	// if testType is 0 then it is a residential test, if testType is 1 then it is a commercial test
	
	while (testDura == 0)
	{
		Button5 = digitalRead(5);
		Button3 = digitalRead(3);
		
		if (!Button3 && (millis()-lastTime)>400) {cycle++; lastTime=millis();}
		if (cycle == 9) {cycle=1;}
		if (!Button5){ testDura=cycle; }

		switch ( cycle ) 
		{
		case 1:
			s7sSendStringSPI(" 20 ");
			break;
		case 2:
			s7sSendStringSPI(" 10 ");
			break;
		case 3:
			s7sSendStringSPI(" 30 ");
			break;
		case 4:
			s7sSendStringSPI("  5 ");
			break;
		case 5:
			s7sSendStringSPI("  4 ");
			break;
		case 6:
			s7sSendStringSPI("  3 ");
			break;
		case 7:
			s7sSendStringSPI("  2 ");
			break;
		case 8:
			s7sSendStringSPI("  1 ");
			break;
		}
	}

	switch ( testDura ) 
	{
	case 1:
		testDura=20;
		break;
	case 2:
		testDura=10;
		break;
	case 3:
		testDura=30;
		break;
	case 4:
		testDura=5;
		break;
	case 5:
		testDura=4;
		break;
	case 6:
		testDura=3;
		break;
	case 7:
		testDura=2;
		break;
	case 8:
		testDura=1;
		break;
	}

	//	delay(50);
	//        detachInterrupt(1);
	//-------------------------------------------------------------------------------------------------------------
	if (!SD.begin(SSSD)) {
		error();
	}
	SPI.setClockDivider(SPI_CLOCK_DIV128);  // Slow down SPI clock

	if (!RTC.begin()) {
		logfile.println("RTC failed");
	}

    //RTC.adjust(DateTime(__DATE__, __TIME__));
    delay(100);
	if (! RTC.isrunning()) {
		// following line sets the RTC to the date & time this sketch was compiled
		// uncomment it & upload to set the time, date and start run the RTC!
		//		RTC.adjust(DateTime(__DATE__, __TIME__));
	} 
    delay(100);
    DateTime now = RTC.now();
    now = RTC.now();
	
    //if (!baro.begin()) {
    //    error1();
    //    delay(1000);

    //}
    // create a new file 
    char  months[3], days[3]; //years[4], months[3], days[3], hours[3], minutes[3], seconds[3];
    sprintf(months, "%02d", now.month());
    sprintf(days, "%02d", now.day());


    char filename[12] = { months[0], months[1], days[0], days[1], '_', '0', '0', '.', 'T', 'X', 'T', '\0' };
    for (uint8_t i = 0; i < 100; i++) {
        //filename[3] = i / 100 + '0';
        filename[5] = (i / 10) % 10 + '0';
        filename[6] = i % 10 + '0';
        if (!SD.exists(filename)) {
            // only open a new file if it doesn't exist

            logfile = SD.open(filename, FILE_WRITE);
            break;  // leave the loop!
        }
    }

	if (! logfile) {
		error();
	}

    logfile.println("Project Number = \t Unknown");
    logfile.println("Project Name = \t Unknown");
    logfile.println("Test Number = \t Unknown");
    logfile.print("Test Description= \t"); 
    if (testType)
    {
        logfile.print(" Commercial");
    }
    else
    {
        logfile.print(" Residential (");
    }
    logfile.print(testDura);
    logfile.println(" min)");
    logfile.print("Date/Time= \t");

    logfile.print(now.year(), DEC);
    logfile.print("/");
    logfile.print(now.month(), DEC);
    logfile.print("/");
    logfile.print(now.day(), DEC);
    logfile.print(" ");
    logfile.print(" \t ");
    logfile.print(now.hour(), DEC);
    logfile.print(":");
    logfile.print(now.minute(), DEC);
    logfile.print(":");
    logfile.println(now.second(), DEC);

    logfile.print("Bucket# \t Density(gpm/(ft^2))");
    logfile.println("\t Date \t Time \t Depth(mm) \t Weight(lbs)");// \t Temperature(*C)");
    logfile.flush();//write the data to the SD card

	attachInterrupt(0, button2, FALLING); //Interrupt connected to digital pin 2 for saving data

	attachInterrupt(1, bNumber, FALLING); //Interrupt connected to digital pin 3 for skipping bucket
	bucket= 1;

    
}

//-----------------------------------------------------------

void loop(void)
{
	DateTime now = RTC.now();
	int j=0;
	int i=0;
	int sensorReading = 0;
	reading = 0;

	float L1 = 0.0, L12=0.0;
	if (testType){L1 = 444.5; L12 = 197580.25;} // mm, mm^2
	else {L1 = 250.825; L12 = 62913.1806;} // mm, mm^2

	boolean released1 = 1;
	writeOK = 0;
	long delta[compNum]={ };
	digitalWrite(greenLEDpin, LOW);//led on the save button goes off
	for (i=0; i<compNum; i++){delta[i]=0;}
	Button1 = digitalRead(button1pin);
	unsigned long timeDelay = 0;
	boolean switchGreen = 1;
    digitalWrite(7, LOW);
	while(!Button1)
	{
        digitalWrite(7, HIGH);
		// delay for the amount of time we want between readings
		reading = 1;
		//delay(dataRate);

		//sensorReading = analogRead(sensorPin);
        int incomingByte = 0;
        //--------------------------------------------------------------------------------
        sensorReading = 0;
        while (incomingByte != 'R'){ incomingByte = Serial.read(); delay(10); }
        while (Serial.available() < 5){ delay(10);}

        // the format of the output from the sensor is :
        // R0000(ASCII 13)
          
        sensorReading += 1000 * (Serial.read() - '0'); delay(10);
        sensorReading += 100 * (Serial.read() - '0'); delay(10);
        sensorReading += 10 * (Serial.read() - '0'); delay(10);
        sensorReading += Serial.read() - '0'; delay(10);
        //Serial.print("sensorReading = "); Serial.println(sensorReading);

        Serial.read();
       
        //--------------------------------------------------------------------------------
        if (sensorReading < 0) sensorReading = 0;
        if (sensorReading > 5000) sensorReading = 5000;
        sensorReading = tripod - sensorReading;
        bool lifted = 0;
        if (sensorReading < -5){ lifted = 1; }
        if (sensorReading < 0) { sensorReading = 0;}
		delta[j]=sensorReading;
		j++;
		if(j==compNum)j=0;
		mean = 0L;
		for (i=0; i<compNum; i++)
		{
			mean += delta[i] * 100L;
		}
		mean /= (compNum*100L);

		digitalWrite(greenLEDpin, LOW);//led on the button 2 goes off

		//-------------------------------------------------------------------------------
		int depth = mean; //mm
		
		float depthf = (float)depth; // mm
		float weight = depthf*(L12+L1*depthf*0.12233+depthf*depthf*0.4988E-2)*2.2e-6; //1.9284e-6; //0.0000022 lbs/mm^3 *1000
		
		int weightint = weight*10.0; //probably should be multiplied by 10: int weightint = weight*10.0; //because for the commertial the weight will be more than 99 lbs
		float density = weight * 0.119828 / (float)testDura;
		int densityint = density*1000.0; //*10.0;
		//-------------------------------------------------------------------------------                

		//SPI.setClockDivider(SPI_CLOCK_DIV64);  // Slow down SPI clock
		if (LWD>2)LWD=0;
		if (LWD == 0)
		{
			// This will output the tempString to the S7S
			if(firstTime)
			{
				s7sSendStringSPI("Dpth");
				setDecimalsSPI(0b00000000);
				delay(500);
				clearDisplaySPI();
				delay(100);
				firstTime = 0;
			}
			sprintf(tempString, "%4d", depth);
			s7sSendStringSPI(tempString);
			setDecimalsSPI(0b00000000);  
		}
		else if(LWD == 1)
		{
			if(firstTime)
			{
				s7sSendStringSPI("-lb-");
				setDecimalsSPI(0b00000000);
				delay(500);
				clearDisplaySPI();
				delay(100);				
				firstTime = 0;
			}    
			sprintf(tempString, "%4d", weightint);
			s7sSendStringSPI(tempString);
			setDecimalsSPI(0b00000100);  
		}
		else if(LWD == 2)
		{
			if(firstTime)
			{
				s7sSendStringSPI("dnst");
				setDecimalsSPI(0b00000000);
				delay(500);
				clearDisplaySPI();
				delay(100);
				firstTime = 0;
			}   
			sprintf(tempString, "%04d", densityint);
			s7sSendStringSPI(tempString);
			setDecimalsSPI(0b00000001);  
		}	
        difference = (abs(sensorReading - mean) * 10000L / mean);

        if (mean < 30)
        {
            percentage = lowPercentage;
        }
        else
        {
            percentage = highPercentage;
        }

		if (difference > percentage) 
		{
			writeOK = 0; 
			switchGreen = 1;
		}
		if (switchGreen && (difference<percentage)) 
		{
            writeOK = 0;
			timeDelay = millis();
			switchGreen = 0;
		}
        if (millis() - timeDelay < 1190) writeOK = 0;
		if((difference<percentage) && (millis()-timeDelay > 1200) && !lifted)
		{
			digitalWrite(greenLEDpin, HIGH); //led on the button 2 goes on

			if(writeOK == 1 && released1==1)
			{
				//fetch the time
				digitalWrite(greenLEDpin, LOW);
				now = RTC.now(); 

				while(previousBucket<bucket)
				{
                    logfile.print(previousBucket, DEC);
                    logfile.print("\t ");
                    logfile.print(" 0 ");
                    logfile.print("\t ");
					logfile.print(now.year(), DEC);
					logfile.print("/");
					logfile.print(now.month(), DEC);
					logfile.print("/");
					logfile.print(now.day(), DEC);
					logfile.print(" ");
					logfile.print(" \t ");
					logfile.print(now.hour(), DEC);
					logfile.print(":");
					logfile.print(now.minute(), DEC);
					logfile.print(":");
					logfile.print(now.second(), DEC);

					logfile.print(" \t ");
					

					logfile.print("\t ");    
					logfile.print(" 0 ");

					logfile.print("\t ");    
					logfile.println(" 0 ");

					

					/*logfile.print("\t ");
					logfile.println(" 0 "); */
					previousBucket++;
				}

                logfile.print(bucket, DEC);

                logfile.print("\t ");
                logfile.print(density , 3);
                logfile.print("\t ");

				logfile.print(now.year(), DEC);
				logfile.print("/");
				logfile.print(now.month(), DEC);
				logfile.print("/");
				logfile.print(now.day(), DEC);
				logfile.print(" ");
				logfile.print(" \t ");
				logfile.print(now.hour(), DEC);
				logfile.print(":");
				logfile.print(now.minute(), DEC);
				logfile.print(":");
				logfile.print(now.second(), DEC);

				logfile.print("\t ");    
				logfile.print(depthf);

				logfile.print("\t ");
				logfile.println(weight);

				/*logfile.print(", ");
				logfile.print(density/100.0,3); */                          
				
				/*float pascals = baro.getPressure();
				float altm = baro.getAltitude();*/
				/*float tempC = baro.getTemperature();*/
				
				/*logfile.print(", ");
				logfile.print(pascals);
				logfile.print(", ");
				logfile.print(altm);*/
				/*logfile.print("\t ");
				logfile.println(tempC);*/

				//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

				logfile.flush();//write the data to the SD card
				bucket++;
				previousBucket=bucket;
				writeOK = 0;
				released1 = 0;
				if (LWD>2)LWD=0;
				for(i=0;i<4;i++)
				{ 
					//SPI.setClockDivider(SPI_CLOCK_DIV64);  // Slow down SPI clock
					
					clearDisplaySPI();
					delay(100);
					if (LWD == 0)
					{
						sprintf(tempString, "%4d", depth);
						s7sSendStringSPI(tempString);
						setDecimalsSPI(0b00000000);  // Sets digit 3 decimal on
					}
					else if(LWD == 1)
					{
						sprintf(tempString, "%4d", weightint);
						s7sSendStringSPI(tempString);
						setDecimalsSPI(0b00000100);  // Sets digit 3 decimal on
					}
					else if(LWD == 2)
					{
						sprintf(tempString, "%04d", densityint);
						s7sSendStringSPI(tempString);
						setDecimalsSPI(0b00000001);  // Sets digit 3 decimal on
					}
					delay(100);
					
				}
				delay(1000);
				for(i=0;i<2;i++)
				{
					clearDisplaySPI();
					delay(100);
					setDecimalsSPI(0b00000000); 
					s7sSendStringSPI("SAUE");  
					delay(100);
				}
				delay(500);
				Button1 = digitalRead(button1pin);
				while(!Button1)
				{
					s7sSendStringSPI("-GO-");
					Button1 = digitalRead(button1pin);
				}
			}

		}

		Button1 = digitalRead(button1pin);
	}

	//-------------------------------------------------------------------
	//----Building the string for showing the current bucket's number----
	tempString[0] = 'b';

	String str;
	char character[2];

	str=String(bucket/100%10);  
	str.toCharArray(character, 2);
	if (character[0] == '0')
	{tempString[1] = ' ';}
	else
	{tempString[1] = character[0];}

	str=String(bucket/10%10);  
	str.toCharArray(character, 2);
	tempString[2] = character[0];
	//
	str=String(bucket%10);  
	str.toCharArray(character, 2);
	tempString[3] = character[0];

	// This will output the tempString to the S7S
	setDecimalsSPI(0b000000);
	s7sSendStringSPI(tempString);
	//-------------------------------------------------------------------
}

// This custom function works somewhat like a serial.print.
//  You can send it an array of chars (string) and it'll print
//  the first 4 characters in the array.
void s7sSendStringSPI(String toSend)
{
	digitalWrite(SSD, LOW);
	for (int i=0; i<4; i++)
	{
		SPI.transfer(toSend[i]);
	}
	digitalWrite(SSD, HIGH);
}

// Send the clear display command (0x76)
//  This will clear the display and reset the cursor
void clearDisplaySPI()
{
	digitalWrite(SSD, LOW);
	SPI.transfer(0x76);  // Clear display command
	digitalWrite(SSD, HIGH);
}

// Set the displays brightness. Should receive byte with the value
//  to set the brightness to
//  dimmest------------->brightest
//     0--------127--------255
void setBrightnessSPI(byte value)
{
	digitalWrite(SSD, LOW);
	SPI.transfer(0x7A);  // Set brightness command byte
	SPI.transfer(value);  // brightness data byte
	digitalWrite(SSD, HIGH);
}

// Turn on any, none, or all of the decimals.
//  The six lowest bits in the decimals parameter sets a decimal 
//  (or colon, or apostrophe) on or off. A 1 indicates on, 0 off.
//  [MSB] (X)(X)(Apos)(Colon)(Digit 4)(Digit 3)(Digit2)(Digit1)
void setDecimalsSPI(byte decimals)
{
	digitalWrite(SSD, LOW);
	SPI.transfer(0x77);
	SPI.transfer(decimals);
	digitalWrite(SSD, HIGH);
}

void button2()//interrupt button (save button)
{
	writeOK = 1;
}

void bNumber()
{
	static unsigned long last_interrupt_time = 0;
	unsigned long interrupt_time = millis();
	// If interrupts come faster than 200ms, assume it's a bounce and ignore
	if ((interrupt_time - last_interrupt_time > 400) && (!reading)) 
	{
		bucket++;
	}
	if ((interrupt_time - last_interrupt_time > 200) && (reading)) 
	{
		LWD++;
		//if (LWD>2)LWD=0;
		firstTime=1;
	}
	last_interrupt_time = interrupt_time;
}
