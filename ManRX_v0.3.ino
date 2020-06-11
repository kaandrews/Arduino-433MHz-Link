
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
   bool blRecord = true;

   // Preamble and clock detection
   unsigned long ulClockDetect = 0;
   int iPreamble = 0;
   int iClock;

   // Read bits
   unsigned long ulLastBit;
   bool blLastBit=false;
   int iChanges = 0;

   // Received packet
   byte byPacket[24] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
   byte byByte = 0;
   byte byBit = 0;
   
   /*
    * States:
    * 0 - Looking for the preamble
    * 1 - Preambe found, waiting for frame start sequence
    * 3 - Found frame start, reading data
    * 4 - Data received, waiting
    */

   public:

   // Constructor
   MANRX(byte pinRF, int Clock): byRFpin(pinRF), iClock(2*Clock) {
    
   }

   // Set up the environment
   void setup(){
    pinMode(byRFpin,INPUT);

    byState = 0;

    blRecord = true;
   }

   void reset(){
    //Clear the buffers
    for (int i=0; i<bufferSize;i++){
      ulBuffer[i] = 0;
      blBuffer[i] = false;
    }

    Serial.println("Starting");

    // Reset the counters and state
    byState = 0;
    ulClockDetect = 0;
    iPreamble = 0;
    iChanges = 0;
    ulLastBit = 0;
    blLastBit = false;
    byByte = 0;
    byBit = 0;
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

    if (blRecord) {
    // Write the current time and pin state to the buffers
    ulBuffer[iWrite] = micros();
    blBuffer[iWrite] = digitalRead(byRFpin);

    // Increment to next position in buffer, correcting for circular buffer
    iWrite = (iWrite + 1) % bufferSize;
    }
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
          // Ignore mid clock changes
          if (ulBuffer[iRead] > (ulLastBit+(ulClockDetect * 0.8)) && ulBuffer[iRead] < (ulLastBit+(ulClockDetect * 1.2))){
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
                  Serial.println("Too many preambles");
                  blOK = false;
                }
              }
            } else {
              // Something went wrong, reset
              Serial.println("Something went wrong, reset");
              blOK = false;
            }
          } else {
            // Should be mid clock transition
            // Count transitions, should be max 1
            iChanges++;

            if (iChanges > 2) {
              Serial.println("Too many changes");
              blOK = false;
            }
          }

          break;
        case 2:
          // Start recording bytes
          // Look for a transition at the correct time
          if (ulBuffer[iRead] > (ulLastBit+(ulClockDetect * 0.8)) && ulBuffer[iRead] < (ulLastBit+(ulClockDetect * 1.2))){
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
              
              if(byByte == 24) {
                // Reached the end of the packet

                // Check the CRC8 value
                byte crc = CRC8(byPacket,23); // Calculate CRC-8 of received frame

                if (crc == byPacket[23]) {
                  byState = 3;
                } else {
                  Serial.println("CRC doesn't match");
                  blOK = false;
                }
              }
              
              ulLastBit = ulBuffer[iRead];
              iChanges = 0;
            } else {
              // Something went wrong, reset
              Serial.println("Something went wrong recording bits, reset");
              blOK = false;
            }
            
          } else {
            // Should be mid clock transition
            // Count transitions, should be max 1
            iChanges++;
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
        reset();
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
    return byPacket[23];
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

MANRX ManRX(byPinRF,500);

void setup() {

  ManRX.setup();
  
  // Initialise the sketch
  Serial.begin(9600);   // Enable serial

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);

  Serial.println("Starting");
  
  attachInterrupt(digitalPinToInterrupt(byPinRF), isrRF, CHANGE); // Enable interrupt for RF
}

void loop() {
  // Run the receiver routine
  ManRX.run();

  // Print the packet details
  if(ManRX.packetReceived()){
    Serial.println("Data Received");
    Serial.print("Destination: ");
    Serial.println(ManRX.destinationAddress());
    Serial.print("Source: ");
    Serial.println(ManRX.sourceAddress());
    Serial.print("Packet Length: ");
    Serial.println(ManRX.packetLength());

    for (int i=0;i<ManRX.packetLength();i++){
      Serial.print("Byte ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(ManRX.data(i));
    }

    Serial.print("As a string: ");

    for (int i=0;i<ManRX.packetLength();i++){
      Serial.print(char(ManRX.data(i)));
    }

    Serial.println();
    
    Serial.print("CRC: ");
    Serial.println(ManRX.crc());

    ManRX.reset();
  }
}

void isrRF() {
  // Call the interrupt routine of the receiver
  ManRX.isr();
}
