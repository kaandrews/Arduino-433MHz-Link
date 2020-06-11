/*
 * Changes to make:
 * 1) Read message ID and allow to detect and ignore repeat transmissions
 *  Could have an array of 255 bytes and store the latest sequence number in the one matching transmitter address
 * 2) Add a better way to read data from frame, maybe another object
 * 3) Add a way to filter according to receive address - done
 */


/*
 * Class Name: ManRX
 * Purpose:
 * Provide a way to receive data sent using a cheap 433 MHz (or similar) transmitter
 * The data is sent with Manchester Coding, so the transmitter and receiver both
 * need to know the clock speed.
 * The maximum number of bytes of data per frame is 20.
 * The last byte of the frame is a CRC8 code so the receiver can check the data consistency.
 * This is envisaged to be used to transmit sensor data from remote nodes back to a hub with the following structure
 * byte 1 - number indicating the type of reading e.g. temperature
 * byte 2 - number of the source e.g. temperature sensor 1
 * byte 3-6 - 4 bytes used to transmit the float data
 * byte 7 - number indicating type of data e.g. humidity
 * byte 8 - number of the source e.g. humidity sensor 1
 * byte 9-12 - 4 byes to transmit float
 * etc
 * 
 * The structure of the data is not fixed, it is an array of bytes that can be used to transmit any data type
 * The length of the packet could also be extended if required
 * This can also be used to send Char arrays (strings) as in the demo sketch
 * 
 * Constructor:
 * MANRX(byte PinRF,int Clock)
 *  - PinRF, the pin that is connected to the transmitter
 *  - Clock, the clock timing in microseconds, tested working from 200 to 1000
 *  
 * Public Methods:
 * void setup()  - configure the environment, call from the main setup in the sketch
 * void restart() - reset the receiver back to the initial condition, called when ready to receive a new frame
 * byte state() - maybe redundant, returns the current state of the receiver e.g. found preamble, found frame start
 * bool packetReceived() - returns true if a packet has been received and passed CRC8 check
 * unsigned long detectedClock() - returns the clock rate as found from the preamble
 * void isr() - must be called from the main sketch change interrupt service routine for the pin PinRF
 * void run() - should be called as frequently as possible from the main sketch loop
 * byte destinationAddress() - get the destination address from the received frame
 * byte sourceAddress() - get the transmitter address from the received frame
 * byte packetLength() - get the number of bytes used in the 20 byte payload
 * byte data(byte byPosition) - get the data from a byte of the payload received
 * byte crc() - get the CRC8 value from the received frame
 * 
 * Private Methods:
 * byte CRC8(const byte *data, byte len) - calculate the CRC8 value of the provided byte array
 * 
 * Improvements to be made:
 * Currently the ISR needs to be manually coded, cannot create an ISR from an instance, find a better way.
 * Provide checking for the destination address and ignore if it doesn't match.
 * Convert to a library
 * Increase the number of bytes to allow a location e.g. temperature, location 1, value, temperature, location 2, value
 * Add a message number, record the previous message numbers from each node when properly received, the ignore repeats
 * Get rid of the lock in state 2, figure out why it freezes sometimes
 */

class MANRX {
  /*
   * Provides the following methods:
   * Scan the RF input looking for the start sequence
   * Detects the frame alignment sequence
   * Reads the incoming data
   * 
   */

   // Cicular buffer used to store incoming data for processing
   static const int bufferSize = 20;
   unsigned long ulBuffer[bufferSize];
   bool blBuffer[bufferSize];

   // Buffer write and read locations
   volatile int iWrite = 0;
   int iRead = 0;

   // Pin connected to RF receive
   byte byRFpin;

   // Process state
   byte byState = 0;
   //bool blRecord = true;

   // Preamble and clock detection
   unsigned long ulClockDetect = 0;
   int iPreamble = 0;
   int iClock;
   
   // Read bits
   unsigned long ulLastBit;
   bool blLastBit=false;
   int iChanges = 0;

   // Address of receiver
   byte byAddress;
   bool blFilter = true;  // If set to true will only receive frames with correct address

   static const int iDataLength = 20; // Length of the user data

   // Received packet
   byte byPacket[iDataLength+4];
   byte byByte = 0;
   byte byBit = 0;
   
   /*
    * States:
    * 0 - Looking for the preamble
    * 1 - Preamble found, waiting for frame start sequence
    * 2 - Found frame start, reading data
    * 3 - Data received, waiting
    */

   public:

   // Constructor
   MANRX(byte pinRF, int Clock, byte address): byRFpin(pinRF), iClock(2*Clock),byAddress(address)  {
   
   }

   // Set up the environment
   void setup(){
    pinMode(byRFpin,INPUT);

    byState = 0;

    // Initialise the array by setting all elements to 0
    for (int i=0;i<sizeof(byPacket);i++){
      byPacket[i] = 0;
    }

    //blRecord = true;
   }

   void restart(){
    //Clear the buffers
    for (int i=0; i<bufferSize;i++){
      ulBuffer[i] = 0;
      blBuffer[i] = false;
    }

    //Clear the received packet
    // Initialise the array by setting all elements to 0
    for (int i=0;i<sizeof(byPacket);i++){
      byPacket[i] = 0;
    }

    //Serial.println("Starting");

    // Reset the counters and state
    iWrite = 0;
    iRead = 0;    
    byState = 0;
    ulClockDetect = 0;
    iPreamble = 0;
    iChanges = 0;
    ulLastBit = 0;
    blLastBit = false;
    byByte = 0;
    byBit = 0;

    //Serial.println("Re-Starting");
   }

   byte state(){
    // Return the currect program state e.g. 0 = looking for preamble
    return byState;
   }

   bool packetReceived(){
    bool blRtn;
    
    // Has a packet been received?
    if (byState == 3){
      blRtn = true;
    } else {
      blRtn = false;
    }

    return blRtn;
   }

   unsigned long detectedClock(){
    // Return the detected clock length, is half the gap found in the preamble
    return ulClockDetect/2;
   }

   void isr(){
    // Called by the main interrupt routine on change of byRFpin

    //if (blRecord) {
    // Write the current time and pin state to the buffers
    ulBuffer[iWrite] = micros();
    blBuffer[iWrite] = digitalRead(byRFpin);

    // Increment to next position in buffer, correcting for circular buffer
    iWrite = (iWrite + 1) % bufferSize;
    //}
   }

   void run(){
    bool blOK = true;
    
    // Check if there is new data
    if (iRead != iWrite) {
      // Act depending on the state of the program
      switch (byState) {
        //case 0:
          // Do nothing
          //break;
        case 1:
          // Reading the bits looking for the frame start sequence e.g. 101010101011
          // Ignore mid clock changes as these are expected

          // If the timing fits a window near the expected timing
          if ((ulBuffer[iRead] - ulLastBit) > (ulClockDetect * 0.8) && (ulBuffer[iRead] - ulLastBit) < (ulClockDetect * 1.2)) {
            // Transition is in the correct time slot
            // Check how many transitions between last bit and this one, should be max 1
            if (iChanges < 2) {
              // Check if 2 1's in a row
              if(blBuffer[iRead] && blLastBit){
                //Serial.println("11");
                // Found the Frame start seq 11
                // Record byte alignment
                ulLastBit = ulBuffer[iRead];
                iChanges = 0;
                byState = 2;
              } else {
                // Store this bit and continue
                ulLastBit = ulBuffer[iRead];
                blLastBit = blBuffer[iRead];
                iChanges = 0;

                //Increment count of bits
                iPreamble++;

                if (iPreamble > 100) {
                  Serial.println("RX Error: 1.1 Too many preambles");
                  blOK = false;
                }
              }
            } else {
              // Something went wrong, reset
              Serial.println("RX Error: 1.2 Too many transitions in a clock cycle");
              blOK = false;
            }
          } else {
            // The timing is either too short or too long for the timing window
            // Could be a mid clock transition or maybe there was no transition during the timing window
            // Count transitions, should be max 1
            iChanges++;

            if (iChanges > 1) {
              Serial.println("RX Error: 1.3 Too many transitions in a clock cycle");
              blOK = false;
            }
          }

          break;
        case 2:
          // Start recording bytes
          // What happens if there are no transitions in the time window??
          // Look for a transition at the correct time

          // If the timing fits a window near the expected timing
          if (ulBuffer[iRead] - ulLastBit > (ulClockDetect * 0.8) && ulBuffer[iRead] - ulLastBit < (ulClockDetect * 1.2)) {
            // Check how many transitions before the correct time window, should be 1 max
            if (iChanges < 2) {
              if (blBuffer[iRead]) {
                bitSet(byPacket[byByte],byBit);
              } else {
                bitClear(byPacket[byByte],byBit);
              }

              byBit++;

              if(byBit == 8) {
                // Reached the end of the byte, move to next
                byBit = 0;
                byByte++;
              }
              
              if(byByte == sizeof(byPacket)) {
                // Reached the end of the packet

                // Check the CRC8 value
                byte crc = CRC8(byPacket,sizeof(byPacket)-1); // Calculate CRC-8 of received frame

                // Check if the CRC8 matches the received value
                if (crc == byPacket[sizeof(byPacket)-1]) {
                  // Should we check the address
                  if (blFilter) {
                    // Check if the address matched
                    if (byPacket[0] == byAddress) {
                      byState = 3;
                    } else {
                      Serial.println("RX Error: 2.1 Address wrong");
                    }
                  } else {
                    byState = 3;
                  }
                } else {
                  Serial.println("RX Error: 2.2 CRC8 doesn't match");
                  blOK = false;
                }
              }
              
              ulLastBit = ulBuffer[iRead];
              iChanges = 0;
            } else {
              // Something went wrong, reset
              Serial.println("RX Error: 2.3 Too many transitions in a clock cycle");
              blOK = false;
            }
            
          } else {
            // The timing is either too short or too long for the timing window
            // Could be a mid clock transition or maybe there was no transition during the timing window
            
            // Count transitions, should be max 1
            iChanges++;

            if (iChanges > 1) {
              Serial.println("RX Error: 2.3 Too many transitions in a clock cycle");
              blOK = false;
            }
          }
          break;
        case 0:
          // Indicate packet has been received and wait
          //Serial.println("Waiting");
          int iReadPrev = iRead - 1;

          // Correct for the circular buffer
          if (iReadPrev < 0){
            iReadPrev = bufferSize - iReadPrev;
          }

          // Calculate clock interval
          unsigned long ulInterval = ulBuffer[iRead] - ulBuffer[iReadPrev];

          // Check if it matches the expected clock
          if (ulInterval > (iClock*0.8) && ulInterval < (iClock*1.2)){
            // Interval could be part of preamble

            // Increment count of correct intervals found
            iPreamble++;

            // Add the interval to the clock detection
            ulClockDetect += ulInterval;

            // If we have reached 10 correct intervals in a row, lock in on the clock signal
            if (iPreamble == 10) {
              // Change to looking for the frame start sequence
              byState = 1;
              //Serial.println(byState);

              // Get the average clock timing from the signal
              ulClockDetect = ulClockDetect / iPreamble;

              // Record the time of this transition
              ulLastBit = ulBuffer[iRead];
            }
          } else {
            // Reset count and clock detection
            iPreamble = 0;
            ulClockDetect = 0;
          }
          break;
      }

      // Increment to next read position, correcting for circular buffer
      if (blOK){
        iRead = (iRead + 1) % bufferSize;
      } else {
        restart();
      }
      
    } else {
      //Serial.println("No data to process");
    }
  }

  byte destinationAddress(){
    return byPacket[0];
  }

  byte sourceAddress(){
    return byPacket[1];
  }

  byte packetLength(){
    return byPacket[2];
  }

  byte data(byte byPosition){
    // Can be 0-19, corresponds to 3-22
    return byPacket[byPosition+3];
  }

  byte crc(){
    return byPacket[sizeof(byPacket)-1];
  }

  private:

  // Calculate the CRC8 of the byte array, used from destination address to end of data
  byte CRC8(const byte *data, byte len) {
    byte crc = 0x00;
    while (len--) {
      byte extract = *data++;
      for (byte tempI = 8; tempI; tempI--) {
        byte sum = (crc ^ extract) & 0x01;
        crc >>= 1;
        if (sum) {
          crc ^= 0x8C;
        }
        extract >>= 1;
      }
    }
    return crc;
  }
};

const byte byPinRF = 2;

MANRX ManRX(byPinRF,500,0);

// Watchdog
unsigned long ulLastFrame;

void setup() {

  ManRX.setup();
  
  // Initialise the sketch
  Serial.begin(9600);   // Enable serial

  pinMode(LED_BUILTIN, OUTPUT); // The LED stays on otherwise..
  digitalWrite(LED_BUILTIN,LOW);

  Serial.println("Ready to receive");
  
  attachInterrupt(digitalPinToInterrupt(byPinRF), isrRF, CHANGE); // Enable interrupt for RF

  ulLastFrame = millis();
}

void loop() {
  // Run the receiver routine
  ManRX.run();

  if (millis()-ulLastFrame > 600000){
    //No frame received for 10 miutes, reset
    Serial.print("Watchdog triggered, state : ");
    Serial.print(ManRX.state());
    Serial.println(" resetting receiver");

    ManRX.restart();
    ulLastFrame = millis();
  }

  // Print the packet details
  if(ManRX.packetReceived()){
    //Serial.println("Received");
    byte byByte = 0;
//    bool bTemp = false;
//    bool bHumid = false;

//    float fTemp = 0.0;
//    float fHumid = 0.0;

    // Extract results
    // While there are bytes to read
    while (byByte < ManRX.packetLength()){

      // Print a line for each data value
      Serial.print("D,");                   // Line starts with D to indicate data
      Serial.print(ManRX.sourceAddress());  // Where the data came from
      Serial.print(",");

      byte byType = ManRX.data(byByte);     // Read the type of data received

      Serial.print(byType);                 // Print the data type e.g. 1 = Temperature
      Serial.print(",");
      byByte++;
      
      Serial.print(ManRX.data(byByte));     // Print the data source e.g. 1 = first temperature sensor
      byByte++;
      
      // Data types:
      // 1 = Temperature 4 byte float
      // 2 = Humidity 4 byte float

      if (byType == 1 || byType == 2) {
        Serial.print(",");
        Serial.print(getFloat(byByte));
        byByte += 4;
      }

      Serial.println(";");
    }

    ulLastFrame = millis();

    // Get ready for the next packet
    ManRX.restart();
  }
}

void isrRF() {
  // Call the interrupt routine of the receiver
  ManRX.isr();
}

float getFloat(byte byStart){
  byte bylByte = byStart;
  byte byData[4];
  
  // Get the temperature bytes
  for (int i=0; i<4; i++){
    byData[i] = ManRX.data(bylByte);
    bylByte++;
  }

  return *(float*) byData;
}
