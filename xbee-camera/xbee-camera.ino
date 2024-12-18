#include "ov7670.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
#include <util/twi.h>

// the camera is mounted in portrait orientation
#define IMAGE_WIDTH 240
#define IMAGE_HEIGHT 320

// define a specific byte sequence to signal the start of a new frame
// this is a little risky, because there is no guarantee that the
// sequence cannot occur in an image, but it will do, because the
// chances are small
#define SYNC_STR "VSY"

// define an array to hold one line (column) of image data
// the number of bytes is IMAGE_HEIGHT*2, because the encoding
// uses 4 bytes for every 2 pixels
uint8_t pixel_data[IMAGE_HEIGHT*2];

// See: https://www.arduino.cc/en/Hacking/PinMapping2560
// mapping of the vertical (frame) synchronization signal pin
// this signal goes high at the start of a new frame
#define VSYNC       (PINA & (1 << 4))   // VSYNC -> PIN 26 (PA4)

// mapping of the horizontal (row) synchronization signal pin
// this signal is high the transmission of a line
#define HREF        (PIND & (1 << 7))   // HREF  -> PIN 38 (PD7)

// mapping of the camera pixel-clock signal pin
// new pixel data is ready at the rising edges of this clock
#define PCLK        (PINA & (1 << 6))   // PCLK  -> PIN 28 (PA6)

// Check if the UART (the device for the serial communication to USB) is busy, i.e.,
// Check the Data Register Empty bit in the USART Control and Status Register 0 A is not set
// Details: https://onlinedocs.microchip.com/oxy/GUID-173AD72D-41FE-4760-A93C-7078A02BD908-en-US-7.1.1/GUID-6BB9ECEA-6542-4810-89C4-99006D9156A6.html
#define UART_BUSY   (!(UCSR0A & (1 << UDRE0)))

//                    --------------------------------
// OV7670 CAMERA PINS |D0 |D1 |D2 |D3 |D4 |D5 |D6 |D7
// ARDUINO BOARD PINS |48 |36 |44 |34 |42 |32 |40 |30
// ATmega2560 PINS    |PL1|PC1|PL5|PC3|PL7|PC5|PG1|PC7
//                    --------------------------------

// The 8-bit pixel data is sent on the OV7670 pins D7 D6 D5 D4 D3 D2 D1 D0
// The following defines get the bits at the right position in the byte, from the ATmega pins
# define D0    ((PINL & 0b00000010) >> 1)
# define D2    ((PINL & 0b00100000) >> 3)
# define D4    ((PINL & 0b10000000) >> 3)
# define D6    ((PING & 0b00000010) << 5)
// D1, D3, D5 and D7  are together in the PINC register and do not need to be shifted
# define D1_D3_D5_D7 ((PINC & 0b10101010))

// The following expression then gets all bits together for a byte of pixel data
#define GET_FRAME_BYTE (D0|D2|D4|D6|D1_D3_D5_D7)

// This program uses the Arduino XBee shield, more info at the link below.
// https://www.arduino.cc/en/Guide/ArduinoXbeeShield

// Configure the next line with a unique ID number for every robot!
#define SELF 42

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

// Initialize the LED light on the board
void led_init(void) {
  // Set the pin mode for the LED light pin to OUTPUT
  pinMode(LED_BUILTIN, OUTPUT);
  // Turn the LED off
  digitalWrite(LED_BUILTIN, LOW);
}

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

// write a single byte, variable dat, on the serial connection (uart)
static inline void serialWriteByte(uint8_t dat) {
  // wait until the UART is available (has completed processing the previous data)
  while (UART_BUSY) {}
  // set byte to transmit into the UART I/O Data Register (UDR0)
  // this will automatically trigger the UART to transmit it
  UDR0 = dat;
}

static void serialWriteString(const char *str) {
  // write a string to the serial connection, byte by byte
  while (*str != 0) {
    serialWriteByte(*str);
    str++;
  }
}

static void waitForVSYNC(){
  // synchronize with the camera start of a new frame
  // See timing diagram in the OV7670 camera data sheet
  // indicated by a pulse on the VSYNC signal
  while (!VSYNC) {} // wait for VSYNC signal to go high
  while (VSYNC) {}  // wait for VSYNC signal to go low
}

static inline void waitForRisingEdgeOnPixelClock() {
  // wait for pixel clock to go low
  while (PCLK){}
  // wait for pixel clock to go high indicating presence of new pixel data
  while (!PCLK){}
}

static void sendPixelDataOnSerialConnection() {
    // set the pointer to the start of the array
    uint8_t *pdi = pixel_data;

    // transmit the entire array
    uint16_t y = sizeof(pixel_data);
    while (y-- != 0) {
      // we need to slow down to not overflow the serial connection buffer
      delayMicroseconds(15);
      // write a byte from the pixel data array
      serialWriteByte(*pdi++);
    }
}

static void captureFrame(uint16_t n_rows, uint16_t n_cols) {
  // capture and transmit a frame of n_rows rows and n_cols columns

  // disable interrupts to avoid timing interference
  noInterrupts();

  // send the frame synchronization sequence 'VSY' so the receiver (frame-viewer)
  // knows this is the start of a new frame
  serialWriteString(SYNC_STR);

  // x counts backward the number of remaining columns
  uint16_t x = n_cols;

  // synchronize with the camera start of a new frame
  waitForVSYNC();

  // for every column of pixels
  while (x != 0) {
    x = x - 1;

    // y counts backward the number of remaining pixels in the column
    uint16_t y = n_rows;

    // pdi is a pointer into the pixel_data array
    uint8_t *pdi = pixel_data;

    // wait for the camera line synchronization signal
    // wait for HREF to go high (indicating start of the line)
    while (!HREF){}

    // for each pixel on the line read two bytes of data into the pixel data array
    while (y-- != 0) {
      // wait for pixel clock rising edge
      waitForRisingEdgeOnPixelClock();
      // move the data to the pixel data array and increment the pointer
      *pdi++ = GET_FRAME_BYTE;
      // wait for pixel clock rising edge
      waitForRisingEdgeOnPixelClock();
      // move the data to the pixel data array and increment the pointer
      *pdi++ = GET_FRAME_BYTE;
    }

    // send the line pixel data to the serial connection
    // note that we have set the camera settings to have an extra long
    // delay before moving to the following line, allowing us to send
    // the data on the serial connection
    sendPixelDataOnSerialConnection();

    // wait for HREF low (end of row)
    while (HREF){}
  }

  interrupts(); // re-enable interrupts
}

void arduinoMegaInit(void) {
  // Set up a 8MHz clock on output pin 46 (it is connected to ov7670's XCLK master clock signal)
  // Although the data sheet says it should be min 10MHz, it seems to work at 8MHz (the highest we can do with our 16MHz clocked microcontroller).
  // We are using Timer5. Note that this potentially creates incompatibility with the servo library,
  // which also uses Timer 5
  // <https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/ATmega640-1280-1281-2560-2561-Datasheet-DS40002211A.pdf>

  // make an output of Pin 46 / PL3
  pinMode(46, OUTPUT);
  
  // Settings of the timer control registers of Timer 5
  // For details see the ATmega data sheet
  // TCCR5A - Timer/Counter 5 Control Register A
  // COM5A1:0 - Compare Output Mode for Channel A. set to 01 - Toggle OCA on compare match
  // WGM5 - Waveform Generation Mode:  Set to 1111
  // WGM51:0 in register A
  TCCR5A = _BV(COM5A0) | _BV(WGM51) | _BV(WGM50);
  // TCCR5B - Timer/Counter 5 Control Register B
  // WGM53:2 in register B
  // CS52:0: Clock Select: 001 = no pre-scaling
  TCCR5B = _BV(WGM53) | _BV(WGM52) | _BV(CS50);
    // OCR5A Output Compare Register 5 A (16 bit)
  // set minimal value for fastest clock
  OCR5A = 0; // (F_CPU)/(2*(X+1)) = 16MHz / 2 = 8 MHz

  // set pin 26 (PA4) as input (is connected to VSYNC)
  pinMode(26, INPUT);
  // set pin 28 (PA6) as input (is connected to PCLK)
  pinMode(28, INPUT);
  
  // set the data pins to input mode
  // OV7670 CAMERA PINS |D0 |D1 |D2 |D3 |D4 |D5 |D6 |D7
  // ARDUINO BOARD PINS |48 |36 |44 |34 |42 |32 |40 |30
  pinMode(48, INPUT);
  pinMode(36, INPUT);
  pinMode(44, INPUT);
  pinMode(34, INPUT);
  pinMode(42, INPUT);
  pinMode(32, INPUT);
  pinMode(40, INPUT);
  pinMode(30, INPUT);

  // Set up twi (two-wire-interface to the OV7670) for 100khz
  // disable pre-scaler for TWI, clear TWPS0 and TWPS1
  TWSR &= 0b11111100; // TWPS1 = 0, TWPS0 = 0
  // Set TWI bit rate
  TWBR = 72; // // set to 100khz = 16Mhz / (16+2*72)

  // set the serial connection settings to the laptop to 1Mbps
  Serial.begin(115200, SERIAL_8N1); // when setting to 250000 I get two images in one frame, with 125000 I get four images in one frame

  // delay to next frame to allow for the camera to adjust to the new settings for the next frame
  waitForVSYNC();

}

void stage1() {
  delay(3000);

  // change only this code ---------
  Serial.println("Hello from camera robot");

  // -------------------------------

  Serial.println("Q");
  delay(3000);
}

void setup(void) {

  // Initialize the XBee wireless module
  xbee_init();
  // Initialize the LED light on the Arduino board
  led_init();

  // setup the microcontroller
  arduinoMegaInit();

  // initialize the camera
  camInit();

  // see: <https://www.electronicscomp.com/datasheet/ov7670-sensor-datasheet.pdf>

  // set the way pixel colors are represented
  // In the yuv422 color space, 2 pixels are encoded in 4 bytes as follows:
  // <Y1><U><Y2><V>
  // first pixel has color <Y0><U><V> in YUV color space
  // second pixel has color <Y1><U><V> in YUV color space
  // see <https://paulbourke.net/dataformats/yuv/>
  setColorSpace(YUV422);

  // set the resolution to QVGA, i.e., 320x240 pixels
  setResolution(QVGA);

  // Write pre-scaler value to register 0x11 (Clock control register) of the camera
  // It determines the speed with which the camera communicates pixels to
  // the microcontroller. Smaller values are faster, but we need to keep up.
  // The value should be between 1 and 63, but if it is too low, it won't work.
  // 8 means that the pixel clock will work at the camera master clock divided by 8, i.e., 8MHz / 8 = 1MHz

  // write value to the Clock control register
  wrReg(0x11, 30); // 8 for 500k, 16 for 250k, 32 for 125k
  wrReg(0x13, 0);

  // Disable Auto Exposure Control (AEC)
  wrReg(0x13, 0x00); // Clear AEC (Bit 0 in COM8)

  // Set Manual Exposure Time
  wrReg(0x07, 0x00); // AECHH: High 6 bits of exposure time (set to 0 for now)
  wrReg(0x10, 0x00); // AECH: Low 8 bits of exposure time (adjust for desired exposure)
  wrReg(0x04, 0x00); // COM1: Lowest 2 bits of exposure time (optional, set to 0)

  // we add dummy pixels to make time in the horizontal 'blanking' to transmit pixel data
  // i.e., we create time between the end of a line and the start of the next line
  // the first (MSB) four bits of register 0x2a and the 8 bits of register 0x2b together
  // form a 12 bit number indicating the number of 'dummy pixels' the camera will produce
  // at the end of each line. This will give us extra time to transmit the pixel data of
  // the line to the serial connection
  // value: binary 010000000000, or decimal 1024 pixels / pixel clock cycles
  // this should be an extra 1.0 millisecond at 1MHz pixel clock
  // we have roughly 10 bit/byte * 320 pixels * 2 bytes/pixel = 6400 bits to transmit
  // on the serial connection, taking about 6.4ms at 1Mbps

  // write the registers
  wrReg(0x2a, 0b10000000);  // 1024 for 500k, 2048 for 115200, 
  wrReg(0x2b, 0b00000000);

  stage1();
}

void loop() {
  // capture frame after frame after frame...
  captureFrame(IMAGE_HEIGHT, IMAGE_WIDTH);
}

