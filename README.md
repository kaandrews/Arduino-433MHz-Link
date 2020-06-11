# Arduino-433MHz-Link
This project is to create a library for Arduino to allow the transmission of data between Arduinos (or similar) using cheap 433 MHz transmitters and receivers.

The included source code allows to send temperature and humidity data from a DHT22 on one Arudino to another Arduino that will display the result by serial connection.
The transmitter and receiver are cheap 433 MHz modules from eBay, I have managed to transmit the signal between floors of my house, have not measured the range.

The data is sent in packets that I have modelled on Ethernet packets, and physically transmitted using Manchster coding.

The packet structure is as below:
3 bytes - Preamble of alternating 1 and 0 to make sure the receiver automatic gain has time to adjust
1 byte - Start frame delimiter, allows to identify the start of the frame after the preamble
1 byte - Destination address, can specify the intended recipient of the packet, of course the transmission can be picked up by any receiver, so it is up to the receiver to filter if required.
1 byte - Source address, allows to identify the sender, for example if you have multiple sensors
1 byte - Length, the length of the data payload
n bytes - Data payload.  The number of data bytes can be adjusted as you need.  In the example I have configured to 20 bytes
1 byte - CRC.  THe last byte is a CRC code that is used by the receiver to confirm receipt

The transmitter randomly chooses a delay between transmissions to try to avoid overlapping with other sensor nodes or regular 433 MHz transmissions e.g. weather stations.

In the current sketches the clock is set to 500us, meaning there will be a bit transmission every 500us.  The receiver should be set to the same clock rate, and will also recover the clock from the signal so long it is within a certain % of the configured clock.

Also included a sample sketch that sends a char string from one node to the other.

Next steps for the project:
1) Provide a way to retransmit messages to try to ensure delivery, together with a message ID so the receiver can filter out already received packets
2) Convert to a library
3) Find a better way to add an interrupt service routine from an object to reduce the user setup required
