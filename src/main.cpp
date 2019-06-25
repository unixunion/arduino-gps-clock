#include <CmdParser.hpp>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <TimeLib.h>
#include <EEPROM.h>

// GPS setup
static const int RXPin = 9, TXPin = 8;
static const uint32_t GPSBaud = 4800;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

// cmd parser
CmdParser cmdParser;

// clock state
bool clock_is_set = false;
int tz_offset = 0;

// EEprom address to save TZ, save with command "TZ 2"
int ee_addr = 0;

// pin which is jumped to Reset
int resetPin = 4;

void setup()
{
    Serial.begin(115200);
    Serial.println("");
    Serial.println(F("Mira's Colour Clock v0.1"));
    Serial.println(F("------------------------"));
    Serial.flush();

    // set reset pin high
    digitalWrite(resetPin, HIGH);
    pinMode(resetPin, OUTPUT);

    // read the tz
    tz_offset = EEPROM.read(ee_addr);
    Serial.print(F("Timezone: GMT ")); Serial.println(tz_offset);
    Serial.flush();

    // GPS
    ss.begin(GPSBaud);
}


void displayInfo()
{
  Serial.print(F("GPS Clock: Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
    
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.println(gps.time.centisecond());
    
  }
  else
  {
    Serial.println(F("INVALID"));
  }
  
}

// do the GPS's work
void gpsWork() {
    while (ss.available() > 0) {
        if (gps.encode(ss.read())) {
            //displayInfo();
            if (clock_is_set && gps.time.minute() != minute()) {
                Serial.println(F("Clock drift, resynchronizing"));
                clock_is_set = false;
            } else if (gps.date.year() > 2018 && !clock_is_set) {
                clock_is_set = true;
                setTime((int)gps.time.hour()+tz_offset, (int)gps.time.minute(), (int)gps.time.second(), (int)gps.date.day(), (int)gps.date.month(), (int)gps.date.year());
                Serial.println(F("Clock is now Syncronized"));
            } else if (!clock_is_set) {
                Serial.print(F("GPS not locked, MCU clock unsynced, sat# ")); Serial.println(gps.satellites.value());
            }
        }
    }
}

void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(F(":"));
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void digitalClockDisplay(){
  // digital clock display of the time
  Serial.print(F("MCU Clock:"));
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(F(" "));
  Serial.print(day());
  Serial.print(F(" "));
  Serial.print(month());
  Serial.print(F(" "));
  Serial.print(year()); 
  Serial.println(); 
}

void reboot() {
    Serial.println("Rebooting...");
    Serial.flush();
    digitalWrite(resetPin, LOW);
    delay(125);
    Serial.print(F("Reboot failed, check pin: ")); Serial.println(resetPin);
}


void loop()
{

    // check GPS is alive
    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
        Serial.println(F("No GPS detected: check wiring."));
        reboot();
    }

    gpsWork();

    // use own buffer from serial input
    CmdBuffer<32> myBuffer;

    //Serial.print("> ");

    if (Serial.available() > 0) {
    // Read line from Serial until timeout
        if (myBuffer.readFromSerial(&Serial, 30000)) {

            if (cmdParser.parseCmd(&myBuffer) != CMDPARSER_ERROR) {
                Serial.print(F("LINE: "));
                Serial.println(myBuffer.getStringFromBuffer());

                Serial.print(F("CMD: "));
                Serial.println(cmdParser.getCommand());

                Serial.print(F("PARM COUNT: "));
                Serial.println(cmdParser.getParamCount());

                const size_t count = cmdParser.getParamCount();
                for (size_t i = 0; i < count; i++) {

                    Serial.print(F("Param "));
                    Serial.print(i);
                    Serial.print(F(": "));
                    Serial.println(cmdParser.getCmdParam(i));
                }

                // show config
                if (cmdParser.equalCommand_P(PSTR("SHOW"))) {
                    Serial.print(F("TZ: ")); Serial.println(tz_offset);
                    Serial.print(F("clock_set: ")); Serial.println(clock_is_set);
                    Serial.print(F("GPS updated: ")); Serial.println(gps.satellites.isUpdated());
                    Serial.print(F("GPS valid: ")); Serial.println(gps.satellites.isValid());
                    Serial.print(F("GPS sat#: ")); Serial.println(gps.satellites.value());
                    displayInfo();
                    digitalClockDisplay();
                }

                // set timezone
                if (cmdParser.equalCommand_P(PSTR("TZ"))) {
                    Serial.print(F(" SETTING TZ: ")); Serial.println(cmdParser.getCmdParam(1));
                    tz_offset = atol(cmdParser.getCmdParam(1));
                    EEPROM.write(ee_addr, tz_offset);
                    clock_is_set = false;
                }
                
                // Check end
                if (cmdParser.equalCommand_P(PSTR("Quit"))) {
                    Serial.println(F("You have written QUIT Command!"));
                }

                // Reboot
                if (cmdParser.equalCommand_P(PSTR("REBOOT"))) {
                    reboot();
                }
                

            } else {
                Serial.println(F("Parser error!"));
            }

        } else {
            Serial.println(F("CMD TIMEOUT!"));
        }
    }
}
// typedef struct command {
// 	char	cmd[10];
// 	void	(*func)	(void);
// };

// void	Cmd_WR () {  // example of one of the commands
// 	/////////////////////////////////////////////////////////
// 	// write a value to target RAM
// 	int addr = GetAddrParm(0);
// 	byte val = GetValParm(1, HEX);
// 	if (SetTargetByte (addr, val) != val)
// 		Serial << "Write did not verify" << endl;	
// }

// command commands[] = {
// 	{"rd",		Cmd_RD},
// 	{"rdw",		Cmd_RDW},
// 	{"wr",		Cmd_WR},
// 	{"l",		Cmd_L},
// 	{"p",		Cmd_P},
// 	{"regs",	Cmd_REGS},
// 	{"io",		Cmd_IO},
// 	{"f",		Cmd_F},
// 	{"sp",		Cmd_SP},
// 	{"per",		Cmd_PER},
// 	{"watch",	Cmd_WATCH},
// 	{"rst",		Cmd_RST},
// 	{"ipc",		Cmd_IPC},
// 	{"set",		Cmd_SET},
// 	{"dig",		Cmd_DIG},
// 	{"an",		Cmd_AN},
// 	{"pin",		Cmd_PIN},
// 	{"pc",		Cmd_PC},
// 	{"info",	Cmd_INFO},
// 	{"/",		Cmd_HELP}
// };



// void	loop() {
// 	char c;

// 	if (Serial.available() > 0) {
// 		c = Serial.read();
// 		if (ESC == c) {
// 			while (Serial.available() < 2) {};
// 			c = Serial.read();
// 			c = Serial.read();	
// 			switch (c) {
// 				case 'A':  // up arrow
// 					// copy the last command into the command buffer
// 					// then echo it to the terminal and set the 
// 					// the buffer's index pointer to the end 
// 					memcpy(cmd_buffer, last_cmd, sizeof(last_cmd));
// 					cmd_buffer_index = strlen (cmd_buffer);
// 					Serial << cmd_buffer;
// 					break;
// 			}
// 		} else {
// 			c = tolower(c);
// 			switch (c) {
				
// 				case TAB:   // retrieve and execute last command
// 					memcpy(cmd_buffer, last_cmd, sizeof(cmd_buffer));
// 					ProcessCommand ();
// 					break;

// 				case BACKSPACE:  // delete last char
// 					if (cmd_buffer_index > 0) {
// 						cmd_buffer[--cmd_buffer_index] = NULLCHAR;
// 						Serial << _BYTE(BACKSPACE) << SPACE << _BYTE(BACKSPACE);
// 					}
// 					break;

// 				case LF:
// 					ProcessCommand ();
// 					Serial.read();		// remove any following CR
// 					break;
				
// 				case CR:
// 					ProcessCommand ();
// 					Serial.read();		// remove any following LF
// 					break;

// 				default:
// 					cmd_buffer[cmd_buffer_index++] = c;
// 					cmd_buffer[cmd_buffer_index] = NULLCHAR;
// 					Serial.print (c);
// 			}
// 		}
// 	}
// }