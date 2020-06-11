/*
 * Next steps:
 * 1 - Provide a better way to add char arrays
 * 2 - Test different clock values for range
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

class MANTX {
  // Frame structure
  // Preamble               configurable number of bytes, to allow receiver to settle
  // Start frame delimited  1 byte
  // Destination address    1 byte
  // Source address         1 byte
  // Length                 1 byte
  // Data                   20 bytes
  // CRC                    1 byte
  
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
  // Data = 3-22
  // CRC = 23
  byte byPacket[24] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

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
    if (byPacket[2] < 20){
      // Add the new data
      byPacket[byPacket[2]+3] = byDataToSend;

      // Increment the counter
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
      byte crc = CRC8(byPacket,23); // Calculate CRC-8
      
      byPacket[23] =  crc;          // Add to the last byte location
      
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
    // Clear all in the frame except addresses
    for (int i=2;i<24;i++){
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
            int iPacketLen = iPreambleLen + 1 + 24;
            
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

const byte bPin = 2;    // pin to transmit RF signal

MANTX ManTX(bPin, 500); // Initialise the object, on pin bPin with 500 miroseconds clock

void setup() {
  Serial.begin(9600);

  // Configure the packet to send
  ManTX.destinationAddress(1);
  ManTX.sourceAddress(100);

  // Send a char array
  char cMsg[] = "Hello World!"; // message to send

  // Add one byte at a time
  for (byte i=0;i<sizeof(cMsg)-1;i++){
    ManTX.addData(cMsg[i]);
  }

  // Add a numerical value
  //ManTX.addData(26);

  // Start sending
  ManTX.sendPacket();

}

void loop() {
  ManTX.run();

  // Check if the frame has finished sending
  if (ManTX.transmitting() == false) {
    Serial.println("Frame Sent");

    // Wait for 30 seconds
    delay(30000);

    //Increment address, just to change something between frames
    ManTX.destinationAddress((ManTX.destinationAddress() + 1) % 255);

    // Clear data from the frame, ready to put in new data
    ManTX.clearData();

    // Add the message again
    // Send a char array
    char cMsg[] = "Hello World! 2!"; // message to send

    // Add one byte at a time
    for (byte i=0;i<sizeof(cMsg)-1;i++){
      ManTX.addData(cMsg[i]);
    }
    
    ManTX.sendPacket();
  }
}
