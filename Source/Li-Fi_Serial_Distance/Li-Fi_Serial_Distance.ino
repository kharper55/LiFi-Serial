/* ====================================================================================================================
 * Li-Fi_Serial_Distance.ino
 * 
 * Description: This source code is intended to be used with the transceiver module wired to the ultrasonic distance
 *              sensor and user input circuitry (push button and potentiometer). So long as it is not held for longer 
 *              than two seconds, upon release, a press of the push button input is used to step messages out over the 
 *              serial link. In the event that the button is held for longer than two seconds, an alternative mode of 
 *              operation is entered in which the Arduino will continuously send a preprocessor determined number of 
 *              messages in sequence, until this number is reached. 
 *              
 *              When the controller board enters this mode of operation, the delay between messages can be adjusted 
 *              via the potentiometer. A mapped ADC integer reading is passed to the Arduino delay function to delay 
 *              the execution of the send loop by up to 1 second. Upon reaching the end of the software defined message 
 *              count, the serial messages can again be stepped by pressing the push button, or the auto-send mode can 
 *              be re-entered with subsequent cycles of two-second presses of the push button. 
 *       
 *              When data is received, it is accepted as a string and chopped up into substrings according to delimiter 
 *              bytes. The trailing half of the message string, which contains a checksum, is compared with a local call 
 *              of the same checksum algorithm passed the received data value. If a mismatch in the two checksums occurs, 
 *              then we know that an error occurred during transmission, and we must take action to adjust some physical 
 *              features of the system to ensure a more ideal optical link.
 *              
 * Pins: The relevant pins can be changed in the preprocessor section below. Pins D2 and D3 are used for 
 *       software serial rx and tx, respectively. Pin D5 is meant to be wired to a push button with external pullup.
 *       Pin A1 is used for an analog reading of a potentiometer wiper pin. Pins D12 and D11 are for the ultrasonic
 *       distance sensor trigger and echo pins, respectively.
 *              
 * Settings: The max distance, push button hold mode change time duration, and predefined message count in the automatic
 *           send mode can be changed in the preprocessor section below. The time delay between subsequent ultrasonic 
 *           pings is set by the global variable pingSpeed.
 * ==================================================================================================================== */
// Library includes
#include <SoftwareSerial.h>    // For software emulated serial on GPIO
#include <Wire.h>              // For I2C on GPIO
#include <Adafruit_SSD1306.h>  // I2C OLED display driver library
#include <NewPing.h>           // Adafruit alt. HCSR04 ultrasonic distance sensor driver library
#include "Timer.h"             // For checking push button high-time duration

// Software serial UART pins
#define rxPin             2    // Software Serial rx
#define txPin             3    // Software Serial tx

// GPIO pins
#define buttonPin         5    // Push button GPIO digital input
#define potPin           A1    // Potentiometer GPIO analog input

// Ultrasonic distance sensor pins
#define TRIGGER_PIN      12   // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PIN         11   // Arduino pin tied to echo pin on ping sensor.
#define MAX_DISTANCE    250   // Maximum distance we want to ping for (in centimeters). Sensor is rated for 400cm.

// Declaration for SSD1306 OLED display. The pins for I2C are defined by the Wire-library. 
#define OLED_RESET       -1   // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C   // See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define SCREEN_WIDTH    128   // OLED display width, in pixels
#define SCREEN_HEIGHT    64   // OLED display height, in pixels

#define MAX_MSG_COUNT 2500    // Max messages to send in automatic send mode of operation
#define ENDTIME 2000          // Set desired end time in mS

// Variable declarations
unsigned int pbState = 0;     // Polling approach GPIO edge detection
unsigned int modeBit = 0;     // Used to control operating mode based on user input
unsigned int pingSpeed = 1;   // How frequently are we going to send out a ping (in milliseconds)
unsigned long pingTimer;      // Holds the next ping time
unsigned int error = 0;       // error represents a count of transmission errors by checksum mismatch  
String RCVDambientLight = ""; // Predeclaration for received data substring
String buff = "";             // Preddeclaration for received message in its entirety (up until '\0')
unsigned int delimPos, endPos;// Predeclarations for delimiter positions of received data string ('*' and '\0')
String RCVDchecksumString;    // Predeclaration for received checksum string and its' integer equivalent
uint16_t RCVDchecksum;
unsigned int TxCount = 0;     // Count of transmitted messages
unsigned int RxCount = 0;     // Count of received messages
unsigned long distance;       // Result of ultrasonic distance measurement

// Object instantiations 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Construct a display object
SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);                   // Initialize and construct a software serial object
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);                       // Construct a sonar object for the HCSR04
Timer timer;                                                              // Setup a timer for measuring pushbutton duration                       
  
// Function Prototypes
uint16_t checksumCalculator(uint8_t* data, uint16_t len);
void toBytes(unsigned char bytesArr[], unsigned int data);
void writeOLEDframe(String &RCVD, unsigned long XMTR);
void echoCheck();
void displayInit();

/* ====================================================================================================================
 * setup()
 * 
 * Description: Setup code for run-time infinite loop. Initialize all GPIO, the OLED display, and both HW (debugging) 
 *              and SW (Li-Fi) serial ports
 * ==================================================================================================================== */
void setup() {
  // Init hardware serial for debugging
  Serial.begin(115200);

  // Init display for displaying
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Init display for this application
  displayInit();
  
  // Define pin modes
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(potPin, INPUT);
    
  // Set the baud rate and begin the operation of SoftwareSerial object
  mySerial.begin(57600);

  // Construct timer object for checking whether or not user wants to enter the automatic sending mode of operation
  Timer timer(MILLIS);
  
  // Start timer for ultrasonic distance pings now
  pingTimer = millis(); 

  // Wait a short duration before executing main to finish inits
  delay(10);
}

/* ====================================================================================================================
 * loop()
 * 
 * Description: Main block for all function calls and control logic.
 * ==================================================================================================================== */
void loop() {
  
  // Start timer on rising edge of digital input press
  if (digitalRead(buttonPin) == 1 && pbState == 0) {
    
    delay(2);
    
    timer.start(); // Begin timer now
    int startTime = timer.read(); // Grab a start time in mS

    while (digitalRead(buttonPin) == 1) {}
    if (timer.read() > startTime + ENDTIME) modeBit = 1;
    else pbState = 1;  
   }
      
   // Step out a serial message on pushbutton falling edge  
   if (digitalRead(buttonPin) == 0 && pbState == 1) {
    
        delay(2);

        if (millis() >= pingTimer) {      // After pingSpeed milliseconds since last ping, do another ping.
          pingTimer += pingSpeed;         // Set the next ping time.
          sonar.ping_timer(echoCheck);    // Send out the ping, calls "echoCheck" function every 24uS where you can check
        }

        // Wait for information to populate the serial buffer
        while (!mySerial.available()) {}
        while (mySerial.available()) {
          // Grab entire string from serial buffer up until a NULL char
          buff = mySerial.readStringUntil('\0'); // Grab entire message

          RxCount++; // Increment the Rx count
          
          // Split received message at delimiters
          delimPos = buff.indexOf('*');                   // Grab index of asterisk in the string
          endPos = buff.indexOf('\0');                    // Grab index of the NULL char in the string
          RCVDambientLight = buff.substring(0, delimPos); // Split the received message into its data and checksum substrings
          RCVDchecksumString = buff.substring(delimPos + 1, endPos);
          
          // Calculate checksum of data obtained in order to check for transmission errors
          RCVDchecksum = RCVDchecksumString.toInt();
          
          // Convert received checksum into an array of bytes for passing to checksumCalculator()
          unsigned char bytesRCVD[4];
          toBytes(bytesRCVD, RCVDambientLight.toInt());
          // Compare the two checksums and increment error in the case of a mismatch
          if (RCVDchecksum != checksumCalculator(bytesRCVD, sizeof(bytesRCVD))) error += 1;
        }
        // Update the OLED display
        writeOLEDframe(RCVDambientLight, distance);
        TxCount++; // Increment the Tx count
        pbState = 0; // Reset the polling GPIO edge-tracking variable 
   }
   // modeBit is set to 1 in the event that the push button input is held for longer than 2 seconds
   if (modeBit == 1) {
    for (int i = 0; i < MAX_MSG_COUNT; i++) {
      // Notice how there's no delays in this sketch to allow you to do other processing in-line while doing distance pings.
      if (millis() >= pingTimer) {    // pingSpeed milliseconds since last ping, do another ping.
        pingTimer += pingSpeed;       // Set the next ping time.
        sonar.ping_timer(echoCheck);  // Send out the ping,
      }                               // calls "echoCheck" function every 24uS where you can check the ping status.
      while (mySerial.available()) {
        // Grab entire string from serial buffer up until a NULL char
        buff = mySerial.readStringUntil('\0');
        RxCount++; // Increment the Rx count
        
        // Split received message at delimiters
        delimPos = buff.indexOf('*');
        endPos = buff.indexOf('\0');
        RCVDambientLight = buff.substring(0, delimPos);
        RCVDchecksumString = buff.substring(delimPos + 1, endPos);
       
        // Calculate checksum of data obtained in order to check for transmission errors
        RCVDchecksum = RCVDchecksumString.toInt();
        // Convert received checksum into an array of bytes for passing to checksumCalculator()
        unsigned char bytesRCVD[4];
        toBytes(bytesRCVD, RCVDambientLight.toInt());

        // Compare the two checksums and increment error in the case of a mismatch
        if (RCVDchecksum != checksumCalculator(bytesRCVD, sizeof(bytesRCVD))) error += 1;
      }
      // Update the OLED display
      writeOLEDframe(RCVDambientLight, distance);
      TxCount++;  // Increment the TxCount
    
      // Delay subsequent executions of the for loop by a scaled amount according to ADC reading of potentiometer wiper
      delay(map(analogRead(potPin),0,1023,0,999));
    }
    // Set modeBit to zero to end the execution of the automatic send mode; require another button hold for restart
    modeBit = 0;
   }
}

/* ====================================================================================================================
 * displayInit()
 * 
 * Description: Initializes display with static text for demonstration. Values are written to the locations just after
 *              the static labels during operation to show the status of the system.
 * ==================================================================================================================== */
void displayInit() {
  display.clearDisplay();
  display.setTextSize(2);           // Prepare font for header text
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  display.println("__TCVR.1__");
  display.setTextSize(1);           // Prepare font for other text
  display.setCursor(0, 32);
  display.println("Tx: ");          // Print location for displaying transmitted value
  display.setCursor(80, 48);
  display.println("CRC?: ");        // Print location for displaying CRC count
  display.setCursor(0, 48);
  display.println("Rx: ");          // Print location for dispalying received value
  display.setCursor(80, 32);
  display.println("#: ");           // Print location for displaying number of transmitted messages
  display.display();
}

/* ====================================================================================================================
 * writeOLEDframe()
 * 
 * Description: Updates the OLED with live information pertaining to the system.
 * ==================================================================================================================== */
void writeOLEDframe(String &RCVD, unsigned long XMTR) {
  display.setCursor(20, 32);
  display.print("        ");   // Ensure next RCVD has a clear background
  display.setCursor(20, 32);
  display.print(XMTR);         // Write the distance
  display.print("cm");
  display.setCursor(20, 48);
  display.print("          "); // Ensure next XMTR has a clear background
  display.setCursor(20, 48);
  display.print(RCVD);         // Write the received data
  display.print('%');
  display.setCursor(110, 48);
  display.print(error);        // Write the error count
  display.setCursor(80, 32);
  display.print("#: ");        // Rewrite the static text for the number sign as a temporary inefficient fix to an
  display.setCursor(93, 32);   // issue with the first received message overwriting the text
  display.print(TxCount + 1);  // Write the transmission count
  display.display();
  display.setCursor(55, 32);
  display.print("      ");
  display.setTextColor(WHITE, BLACK);
}

/* ====================================================================================================================
 * echoCheck()
 * 
 * Description: Measures the distance via the HCSR04 ultrasonic distance sensor. Adapted from the New Ping library.
 *              Timer2 interrupt calls this function every 24uS where you can check the ping status.
 * ==================================================================================================================== */
void echoCheck() { 
  if (sonar.check_timer()) {   // Check to see if the ping was received.
    distance = sonar.ping_result / US_ROUNDTRIP_CM; // Calculate distance
    unsigned char bytes[4];
    toBytes(bytes, distance);
    uint16_t checksum = checksumCalculator(bytes, sizeof(bytes)); // Calculate checksum
    
    // Write message to software serial tx pin
    mySerial.print(distance);  // Write data
    mySerial.print('*');       // Write delimiter to separate the data from the checksum
    mySerial.print(checksum);  // Write the checksum
    mySerial.print('\0');      // Signify the end of the string with a NULL char
  }
}

/* ====================================================================================================================
 * checksumCalculator() - https://www.tutorialspoint.com/cyclic-redundancy-check-crc-in-arduino
 * 
 * Description: Calculates a checksum for a four index array of bytes. Return the 16-bit CRC for an array of bytes. 
 *              The implementation is Fletcher's checksum
 * ==================================================================================================================== */
uint16_t checksumCalculator(uint8_t* data, uint16_t len) {
   // Initialize the checksum as 0x0000 (arbitrary but opposing end must match)
   uint16_t curr_crc = 0x0000;
   // Initialize sum1 as the lower byte and sum2 as the higher byte of the initial value
   uint8_t sum1 = (uint8_t)curr_crc, sum2 = (uint8_t)(curr_crc >> 8);
   int index;
   for(index = 0; index < len; index = index+1) {
      sum1 = (sum1 + data[index]) % 255; // Add successive data bytes to sum1 keeping sum1 < max representable int
      sum2 = (sum2 + sum1) % 255; // Add sum1 values to sum2 keeping sum2 < max representable int
   }
   return (sum2 << 8) | sum1; // Return a 16-bit number, with sum2 as the higher byte and sum1 as the lower byte
}

/* ====================================================================================================================
 * toBytes()
 * 
 * Description: Converts an integer input into an array of 4 chars (bytes). Used for computing checksums.
 * ==================================================================================================================== */
void toBytes(unsigned char bytesArr[], unsigned int data) {
   bytesArr[0] = (data >> 24) & 0xFF;  // Grab the highest order digit, store in array slot 0 
   bytesArr[1] = (data >> 16) & 0xFF;  // Grab the second highest order digit, store in array slot 1 
   bytesArr[2] = (data >> 8) & 0xFF;   // Grab the third highest order digit, store in array slot 2 
   bytesArr[3] = data & 0xFF;          // Grab the fourth highest order digit, store in array slot 3
}

/* ====================================================================================================================
 * ///////////////////////////////////////////////////// END FILE /////////////////////////////////////////////////////
 * ==================================================================================================================== */
