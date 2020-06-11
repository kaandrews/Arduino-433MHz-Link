/*
 * Next steps:
 * 1 - Provide a better way to add char arrays
 * 2 - Test different clock values for range
 * 3 - Add a message ID and a way to retransmit
 */

/*
 * Class Name: ManTX
 * Purpose:
 * Provide a way to send data using a cheap 433 MHz (or similar) transmitter
 * The data is sent with Manchester Coding, so the transmitter and receiver both
 * need to know the clock speed.
 * The maximum number of bytes of data per frame is 20.
 * A CRC8 code is added to the data so the receiver can check the data consistency.
 * This is envisaged to be used to transmit sensor data back to a hub.
 * For example byte 1 indicates the sensor type, byte 2 (or more) include the reading e.g. temperature
 * So long as the receiving sketch understands the format then any data types can be sent 1 byte at a time.
 * This can also be used to send Char arrays (strings) as in the demo sketch
 * 
 * Constructor:
 * MANTX(byte PinRF,int Clock)
 *  - PinRF, the pin that is connected to the transmitter
 *  - Clock, the clock timing in microseconds, tested working from 200 to 1000
 *  
 * Public Methods:
 * void destinationAddress(byte byAddress) - set the address of the receiver
 * byte destinationAddress() - find out what destination address is set
 * void sourceAddress(byte byAddress) - set the address of the transmitter
 * byte sourceAddress() - get the transmitter address
 * bool transmitting() - returns true is data is currently being transmitted
 * byte dataLength() - returns the number of bytes currently added to the frame
 * byte addData(byte byDataToSend) - add a byte to the frame in the next available slot, returns -1 if the frame is full
 * byte sendPacket() - Calculate CRC8 and start sending the frame
 * void clearData() - clears the data from the frame, keeps addresses
 * void run() - should be called as frequently as possible from the main sketch loop
 * 
 * Private Methods:
 * byte CRC8(const byte *data, byte len) - calculate the CRC8 value of the provided byte array
 * 
 * Improvements to be made:
 * Provide a way to add different data types to the frame e.g integer, double, etc.
 * Convert to a library
 */
#include <SimpleDHT.h>

class MANTX {
  // Frame structure
  // Preamble               configurable number of bytes, to allow receiver to settle
  // Start frame delimited  1 byte
  // Destination address    1 byte
  // Source address         1 byte
  // Length                 1 byte
  // Data                   according to constant iDataLength bytes
  // CRC                    last byte
  
  // Pins to configure
  const byte byRfPin;

  // RF transmit data
  const byte byPreamble = B01010101;  // Preamble
  const byte byFSF = B11010101;       // Frame start sequence

  // Transmitter variables
  int iClock;             // Clock interval in microseconds
  int iPreambleLen = 3;   // How many time to send the preamble byte
  bool blClock = false;   // State of the clock, true=high, false=low

  // Array to hold frame
  // Destination address = 0
  // Source address = 1
  // Length = 2
  // Data = set by constant iDataLength
  // CRC = last byte
  
  static const int iDataLength = 20; // Length of the user data field
  
  byte byPacket[iDataLength+4];

  // Flags
  bool blDest = false;      // Has the destination address been set
  bool blSource = false;    // Has the sender address been set
  bool blTransmit = false;  // Are we transmitting now

  // Variables
  byte byByte;              // Current byte being transmitted
  byte byBit;               // Current bit being transmitted
  unsigned long ulLast;     // Used to check if the clock interval has been reached

  public:

  // Constructor
  MANTX(byte PinRF,int Clock):byRfPin(PinRF),iClock(Clock) {
    // Assigns the transmitter pin and clock interval
  }

  // Setup - call from Sketch setup
  setup() {
    // initialise the TX pin
    pinMode(byRfPin,OUTPUT);

    // Initialise the array by setting all elements to 0
    for (int i=0;i<sizeof(byPacket);i++){
      byPacket[i] = 0;
    }
  }

  // Set the destination address
  void destinationAddress(byte byAddress){
    byPacket[0] = byAddress;
    blDest = true;
  }

  byte destinationAddress(){
    return byPacket[0];
  }

  // Set the source address
  void sourceAddress(byte byAddress){
    byPacket[1] = byAddress;
    blSource = true;
  }

  byte sourceAddress(){
    return byPacket[1];
  }

  // Is the transmitter active now
  bool transmitting() {
    return blTransmit;
  }

  // How many data bytes have been added
  byte dataLength(){
    return byPacket[2]+1;
  }

  // Add data to send, up to 20 bytes can be added
  // Returns -1 if failed
  byte addData(byte byDataToSend){
    byte byRtn = 0;

    // If the current data length is less than 20 bytes
    if (byPacket[2] < iDataLength){
      // Add the new data to the next available location
      byPacket[byPacket[2]+3] = byDataToSend;

      // Increment the size field
      byPacket[2]++;
    } else {
      // Unable to add data
      byRtn = -1;
    }

    return byRtn;
  }

  byte sendPacket(){
    
    byte byRtn=0;

    // Check if the addresses are set and there is data to send
    if (blDest && blSource && byPacket[2] > 0){
      byte crc = CRC8(byPacket,sizeof(byPacket)-1); // Calculate CRC-8 for addresses,size and data fields
      
      byPacket[sizeof(byPacket)-1] =  crc;          // Add to the last byte location
      
      // Start transmitting
      blTransmit = true;  // Start transmitting in run()
      byByte = 0;         // Start at the first byte
      byBit = 0;          // Start at the first bit
      ulLast = micros();  // Record the start time
    } else {
      byRtn = -1;         // Return -1 if we can't transmit
    }

    return byRtn;
  }

  void clearData(){
    // Clear all in the frame except sender and destination addresses
    for (int i=2;i<sizeof(byPacket);i++){
      byPacket[i] = 0;
    }
  }

  void run() {
    // Check if we should be transmitting
    if (blTransmit) {
      // Check if the clock has ticked
      if (micros()-ulLast > iClock){
        // If yes invert the clock state
        blClock = !blClock;

        // Check which byte we are transmitting
        if (byByte < iPreambleLen) {
          // Send the requested number of preamble bytes
          // If user enters 1 as preamble length, starts at 0, transmits 1st byte
          
          digitalWrite(byRfPin, bitRead(byPreamble,byBit)^blClock);

        } else if (byByte == iPreambleLen) {
          // Send the frame start delimited byte
          // If user enters 1 as preamble, preamble = byte 0, FSD = byte 1
          
          digitalWrite(byRfPin, bitRead(byFSF,byBit)^blClock);

        } else if (byByte > iPreambleLen) {
          // Send the frame byte array
          // Includes addresses, length, data and CRC
          
          // If user enters 1 as preamble, preamble = b0, FSD = b1, data from B2 to B25
          // Corresponds to array 0 to 23, so byte - preamble length(1) - 2

          digitalWrite(byRfPin, bitRead(byPacket[byByte-iPreambleLen-1],byBit)^blClock);

        }

        // If the clock has moved to low
        if (!blClock){
          if (byBit == 7){
            // If we have reached the last bit in the byte
            
            // Check if we have reached the end of the bytes
            int iPacketLen = iPreambleLen + 1 + sizeof(byPacket);
            
            if (byByte == iPacketLen) {
              // If we are at the end stop transmitting
              blTransmit = false;
              digitalWrite(byRfPin, LOW);

            } else {
              // otherwise move to the next byte
              byBit = 0;
              byByte++;
            }
          } else {
            // Increment to the next bit
            byBit++;
          }
        }

        // Update the clock
        ulLast = micros();
      }
    }
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

// Addresses
const byte bySource = 2;
const byte byDest = 0;

// Pins used
const byte bPin = 2;    // pin to transmit RF signal
const byte bPinDHT22 = 3;

// Timer
unsigned long ulLastSend;
unsigned long ulDelay = 3000;  // Start with 3 second delay then move to average 5 minutes between transmissions

MANTX ManTX(bPin, 500); // Initialise the object, on pin bPin with 500 miroseconds clock
SimpleDHT22 dht22(bPinDHT22);
//SimpleDHT11 dht22(bPinDHT22);

void setup() {
  Serial.begin(9600);
  Serial.println("Starting");

  ManTX.setup();

  // Configure the addresses of the frame to send
  ManTX.destinationAddress(byDest);
  ManTX.sourceAddress(bySource);

  randomSeed(analogRead(0));

  ulLastSend = millis();
}

void loop() {
  ManTX.run();

  // Read the DHT22 values after ulDelay milliseconds
  if (millis() - ulLastSend > ulDelay) {

    //Serial.println("Time");
    // Try to get the result from the DHT22 as float values
    float fTemp = 0.0;
    float fHumidity = 0.0;

    int err = SimpleDHTErrSuccess;

    if ((err = dht22.read2(&fTemp, &fHumidity, NULL)) != SimpleDHTErrSuccess) {
      // Failed to read the result
      Serial.print("Read DHT22 failed, err=");
      Serial.println(err);
      // Retry in 2.5 seconds
      ulDelay = 2500;
    } else {

      Serial.print("Temperature: ");
      Serial.print(fTemp);
      Serial.print("*C, Humidity: ");
      Serial.print(fHumidity);
      Serial.println(" %");
      
      // Enter the values into the frame
      ManTX.clearData();

      // first value is temperature float
      ManTX.addData(1); // 1 means temperature value
      ManTX.addData(1); // Source e.g. temperature monitor 1

      byte byData[4];

      // Convert the float temperature to array of bytes
      *(float *) byData = fTemp;

      // Add temperature to packet
      for (int i=0;i<4;i++) {
        ManTX.addData(byData[i]);
      }

      // Second value is humidity
      ManTX.addData(2); // 2 means humidity
      ManTX.addData(1); // Source 1

      // Convert the float humidity into array of bytes
      *(float *) byData = fHumidity;

      // Add temperature to packet
      for (int i=0;i<4;i++) {
        ManTX.addData(byData[i]);
      }

      // To decode back to a float on the other end
      //float fTempor = *(float*) byData;

      // Send frame
      ManTX.sendPacket();

      // Randomize time to send next result to reduce clashes
      // 4 minutes + 3-9 x 10 seconds
      ulDelay = 240000 + (10000 * random(3,10));

      Serial.println("Sent");
      
    }

    ulLastSend = millis();
  }
}
