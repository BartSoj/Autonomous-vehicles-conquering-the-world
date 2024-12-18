// This program uses the Arduino XBee shield, more info at the link below.
// https://www.arduino.cc/en/Guide/ArduinoXbeeShield

// Configure the next line with a unique ID number for every robot!
#define SELF 43

// Define the PAN (Personal Area Network) number
// It must be unique for every team and the same for all robots in one team!
// For team number NN use: "A0NN"
#define PAN_ID "A009"

// Define a channel ID to use for communication
// It must be the same for all robots in one team!
// It is represented by a 2-digit hexadecimal number between 0B and 1F.
#define CHANNEL_ID "12"

// Some macros needed for the xbee_init function. Do not touch :-).
#define STRING(name) #name
#define TOSTRING(x) STRING(x)

// The xbee_init function initializes the XBee Zigbee module
// When the program is running, the switch on the wireless proto shield should
// be in the position 'MICRO'. During programming of the Arduino, the switch
// should be in the position 'USB'. It will only work if the XBee module is set
// to communicate at 9600 baud. If it is not, the module needs to be
// reprogrammed using USB XBee dongle and the XCTU program.
// Note that you may need to reset the Arduino after moving the switch to 'MICRO'.
void xbee_init(void) {
  Serial.flush();      // make sure the buffer of the serial connection is empty
  Serial.print("+++"); // this brings the XBee module in its command mode
  // see https://cdn.sparkfun.com/assets/resources/2/9/22AT_Commands.pdf
  delay(2000); // wait two seconds for the module to go in command mode
  // now we can use AT commands to program it
  Serial.print("ATCH " CHANNEL_ID "\r");     // set the channel to CHANNEL_ID
  Serial.print("ATID " PAN_ID "\r");         // set the network PAN ID to PAN_ID
  Serial.print("ATMY " TOSTRING(SELF) "\r"); // set the network ID to SELF
  Serial.print("ATDH 0000\rATDL FFFF\r");    // broadcast messages to all nodes
  Serial.print("ATAP 0000\r");               // API disabled
  Serial.print("ATCN\r"); // exit command mode and return to transparent mode,
                          // communicate (broadcast) all data on the serial link
                          // onto the wireless network
}

// This function is called only once at the start of the program
void setup(void) {
  // Initialize the Serial communication
  Serial.begin(115200);  // set the baud rate to match the one of the xbee module
  // Initialize the XBee wireless module
  xbee_init();
}

// This function is called over and over again when the program is running
void loop(void) {
  // Check if there is some incoming data from the wireless network
  if (Serial.available() > 0) {
    // Uncommenting the Serial.print / Serial.write lines below will echo the
    // incoming data back onto the network,
    /* This could be useful for debugging, because the incoming data will not
       show up on the monitor in the Arduino IDE, but the data that is sent
       will. Note that if you have multiple robots in the network, these echos
       are going to ping pong across the network!!
    */
    // Serial.print("Echo incoming data: ");

    // As long as there is more data in the input buffer...
    while (Serial.available() > 0) {
      // Declare a variable to hold the data coming in from the serial link
      // Note that it is a number (int 0-255), characters are usually encoded to
      // numbers using ASCII codes
      int incomingByte = 0;
      // Read the incoming data from the serial connection
      incomingByte = Serial.read();
      Serial.print("\nIncoming message received\n");
      Serial.print(incomingByte);

      /* Echo it back to the serial connection.
         Note that we use 'write' and not 'print', because we do not want it to
         show up as a readable version of the number, but as the original
         character.
      */
      // Serial.write(incomingByte);
    }
    // End the echoing of the incoming data with a new line
    // Serial.println("");
  }
}
