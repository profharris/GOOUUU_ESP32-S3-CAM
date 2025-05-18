# GOOUUU_ESP32-S3-CAM
ESP32-S3-CAM    (GOOUUU ESP32-S3-CAM)
```
ESP32-S3-CAM    (GOOUUU ESP32-S3-CAM)

(Same pinout as ESP32-S3-WROOM (CAM), & ESP32-S3-DevKitC-1)
(Camera pinout same as:  CAMERA_MODEL_ESP32S3_EYE  camera_pins.h)

Xtensa® 32-bit          11-pins wide×20-pins
dual-core LX7               ESP32-S3-CAM      ESP32-S3-WROOM-1 N16R8
240MHz, 512KB SRAM      • • • • • • • • • • •
16MB Flash, 8MB PSRAM      _______________
                          |  ___   _   __¯|      IO##~   SD-Card
    DVP Camera IO##_      | | | |_| |_|   |      IO##*   PSRAM
                       .——| | |           |——.                         SPI LCD       SPI LCD 7/8-Pins
                3V3   1|•:|————————————/:@|:¤|40 IO43 TX›[U0TXD LED]   —————————  1. GND                  Brown
[RESET    ]      EN   2|•:|  .··. .    '——|:o|39 IO44 RX‹[U0RXD    ]     -1 MISO  2. VCC                  Red
[CAM_SIOD ]SDA  IO4_  3|•:|  WiFi ß       |:o|38 IO1  SDA[A0       ]   IO42 SCLK  3. SCL  (SCLK)          Orange
[CAM_SIOC ]SCL  IO5_  4|•:|  ˜¨¨˜ °       |:o|37 IO2  SCL[A1       ]   IO41 MOSI  4. SDA  (MOSI)          Yellow
[CAM_VSYNC]     IO6_  5|•:|ESP32-S3-N16R8 |:o|36 IO42    [     SCLK]—  IO0  RST   5. RES  (RESET)         Green
[CAM_HREF ]     IO7_  6|•:|               |:o|35 IO41    [     MOSI]—  IO45 DC    6. DC   (Data/Command)  Blue
[CAM_XCLK ]    IO15_  7|•:|  ŒÆ     F©    |:•|34 IO40~   [SD_DATA •]   IO47 CS    7. CS   (Chip Select)   Violet
[CAM_Y9   ]D7  IO16_  8|•:'———————————————':•|33 IO39~   [SD_CLK  •]   IO21 BL*  (8) BLK  (BackLight)     Grey
[CAM_Y8   ]D6  IO17_  9|• ._______________. •|32 IO38~   [SD_CMD  •]
[CAM_Y7   ]D5  IO18_ 10|• | ::::::::::::: | •|31 IO37*   [PSRAM   •]              -1 MISO (N/C)
[CAM_Y4   ]D2   IO8_ 11|• |    Camera     | •|30 IO36*   [PSRAM   •]
[  Switch ]     IO3  12|o ¯¯:::::::::::::¯¯ •|29 IO35*   [PSRAM   •]
[  Shutter]    IO46  13|o   ESP32-S3-CAM    o|28 IO0     [BOOT  RST]—  Switch/
[CAM_Y3   ]D1   IO9_ 14|•   ¬ ¬  PWR¤ ¤ [¤] o|27 IO45    [       DC]—  Shutter
[CAM_Y5   ]D3  IO10_ 15|•   ¨ ¨......TX  48 ¤|26 IO48 RGB[WS2812   ]   —————————
[CAM_Y2   ]D0  IO11_ 16|• RST |CH304G | BOOTo|25 IO47    [       CS]—  IO3  SW
[CAM_Y6   ]D4  IO12_ 17|• [Ø]  '''''''  [Ø] o|24 IO21    [      BLK]-  IO46 SH
[CAM_PCLK ]    IO13_ 18|• .......O T....... ø|23 IO20 D— [A19      ]
[A13      ]    IO14  19|o | USB |T T| USB | ø|22 IO19 D+ [A18      ]   I²C QWIIC
                5V0  20|• |  C  |G L|  C  | •|21 GND                   —————————
                       '——'ESP32'———'CH343'——'   IO48 RGB NeoPixel    1  *  GND
                                                                      2  *  3V3
ESP32-S3 Pins: 0…18 GPIO, 19…20 D+/D-, 21 GPIO, 22…25 Do Not Exist,   3 IO1 SDA
26…32 QSPI ƒlash, 33…34 Missing, 35…42 GPIO, 43…44 TX/RX, 45…48 GPIO  4 IO2 SCL

————————————————————————————————————————————————————————————————————
Product Features:
--------------------------------------------------------------------
 1. ESP32-S3-CAM is designed based on the “ESP32-S3-WROOM-1 N16R8”,
    with PCB on-board antenna & an IPEX external antenna connection.
 2. ESP32-S3 development board can be paired with OV2640/OV5640 two
    kinds of cameras optional.
 3. ESP32-S3 is a low-power MCU system-on-chip (SoC) with integrated
    2.4GHz WiFi & low-power dual-mode Bluetooth.
 4. ESP32-S3 has a complete WiFi subsystem and low-power Bluetooth
    subsystem, with industry-leading low-power performance and RF
    performance, and supports a variety of low-power operating states
    to meet the power requirements of various application scenarios.
 5. ESP32-S3 chip provides rich peripheral interfaces, and has a
    variety of unique hardware security mechanisms, perfect security
    mechanisms to enable the chip to meet strict security requirements.
 6. Support AI acceleration, accelerating neural network computing.
 7. High-performance Image Recognition, voice awakening & recognition, 
    ESP-WHO and ESP-SKAINET.
 8. Equipped with ultra-low-power consumption processor (ULP).
 9. Supports Flash encryption based on the AES-XTS algorithm, security 
    start-up, digital signature and HMAC based on RSA algorithms.
10. The ESP32-S3 also adds a “World Controller” module, which provides 
    two non-interference work environments to achieve a trusted work 
    environment or permissions separation mechanism.

Performance parameters:

 Controller: Xtensa dual-core 32-bit LX7 CPU, frequency up to 240MHz
    Storage: 384KB of ROM, 512KB SRAM, 16KB RTCSRAM, 8MB PSRAM
Temperature: -40 to 65 degrees Celsius
      Power: 3V to 3.6V.
   45 GPIOs: 2×12-bit ADC (up to 20 channels)
             2×I²C interfaces
             2×I²S interfaces
             4×SPI interfaces
             3×UART interfaces
             1×USB OTG interface.

WIFI:

o Supports IEEE 802.11b/g/n protocols
o Supports 20MHz and 40MHz bandwidths in the 2.4GHz band
o Supports 1T1R mode with data rates up to 150Mbps

WIRELESS MULTIMEDIA (WMM):

o Frame aggregation (TX/RX A-MPDU, TX/RX A-MSDU)
o ImmediateBlock ACK
o Fragmentation/defragmentation o Automatic beacon monitoring
  (hardware TSF)
o 4xvirtual WiFi interfaces
o Simultaneous support for infrastructure BSS Station mode, SoftAP
  mode & Station+SoftAP hybrid mode. Please note that when ESP32-S3
  is scanning in Station mode, the SoftAP channel will be changed at
  the same time.
o Antenna Diversity
o 802.11 mc FTM
o External power amplifier support

BLUETOOTH:

o Bluetooth Low Power BLE (Bluetooth LE), Bluetooth5, Bluetooth Mesh
o High power mode (20 dBm, shared PA with WiFi)
o Rate support: 125Kbps, 500Kbps, 1Mbps, 2Mbps.
o AdvertisingExtensions
o MultipleAdvertisement Sets (MultipleAdvertisement Sets)
o Channel Selection Algorithm (Channel Selection Algorithm #2)
o WiFi and Bluetooth coexist and share the same antenna.

ADVANCED PERIPHERAL INTERFACES AND SENSORS:

 45×GPIO ports
o 4×SPI
o 1×LCD interface (8-bit)
o 1×16-bit parallel RGB, L8080, MOTO6800), support RGB565, YUV422,
    YUV420, YUV411  & conversion between each other
o 1×DVP8-bit~16-bit camera interface
o 3×UART
o 2×I²C
o 2×I²S
o 1×RMT (TX/RX)
o 1×Pulse Counter
o 1×LED PWM Controller, up to 8 channels
o 1×Full Speed USB OTG
o 1×USB Serial/JTAG Controller
o 1×MCPWM
o 1×SDIO host interface with 2 card slots
o 1×General Purpose DMA Controller (GDMA for short),
  5 receive channels and 5 transmit channels
o 1×TWAI controller, IS011898-1 compatible (CAN specification 2.0)

ANALOG INTERFACES:

o 1×12-bit SAR ADC, up to 20 channels
o 1×internal Temperature sensor
o 14xCapacitive sensing (Touch) GPIOs

Timers:

o 4×54-bit General-Purpose timers
o 1×52-bit System timer
o 3×Watch timers

————————————————————————————————————————————————————————————————————
GPIO26 to GPIO32 are connected to the integrated SPI Flash and PSRAM
and are not recommended for other uses. They are not exposed in this
particular board, but if they are exposed on your board, avoid using
them:

    GPIO26 (Flash/PSRAM SPICS1)
    GPIO27 (Flash/PSRAM SPIHD)
    GPIO28 (Flash/PSRAM SPIWP)
    GPIO29 (Flash/PSRAM SPICS0)
    GPIO30 (Flash/PSRAM SPICLK)
    GPIO31 (Flash/PSRAM SPIQ)
    GPIO32 (Flash/PSRAM SPID)

————————————————————————————————————————————————————————————————————
SDcard Pins:
————————————
    An SDcard slot is integrated on the back of the ESP32-S3-WROOM-1
    board. We can use GPIO38-GPIO40 of the ESP32-S3-WROOM-1 to drive
    the TF SD-card.

    The TF SDcard of ESP32S3-WROOM uses SDMMC, a ‘1-bit bus’ driving
    method, which has been integrated in the Arduino IDE, and we can
    call the "SD_MMC.h" library to drive it. For details, see the
    SDcard chapter in the FreeNov tutorial.

————————————————————————————————————————————————————————————————————
Analog to Digital Converter (ADC)

The ESP32 has 20x 12-bit ADC input channels. These are the GPIOs
that can be used as ADC and respective channels:

    o ADC1_CH0 (GPIO 1)
    o ADC1_CH1 (GPIO 2)
    o ADC1_CH2 (GPIO 3)
    o ADC1_CH3 (GPIO 4)
    o ADC1_CH4 (GPIO 5)
    o ADC1_CH5 (GPIO 6)
    o ADC1_CH6 (GPIO 7)
    o ADC1_CH7 (GPIO 8)
    o ADC1_CH8 (GPIO 9)
    o ADC1_CH9 (GPIO 10)
    o ADC2_CH0 (GPIO 11)
    o ADC2_CH1 (GPIO 12)
    o ADC2_CH2 (GPIO 13)
    o ADC2_CH3 (GPIO 14)
    o ADC2_CH4 (GPIO 15)
    o ADC2_CH5 (GPIO 16)
    o ADC2_CH6 (GPIO 17)
    o ADC2_CH7 (GPIO 18)
    o ADC2_CH8 (GPIO 19)
    o ADC2_CH9 (GPIO 20)

The ADC input channels have a 12-bit resolution. This means that you
can get Analog readings ranging from 0 to 4095, in which a 0 read
corresponds to 0V and 4095 to 3.3V. You can also set the resolution
of your channels on the code and the ADC range.

====================================================================
/* ESP32-S3-rgbFade
   ESP32-S3-WROOM (CAM Module)

Make an RGB LED display a rainbow of colors!

The RGB LED on the ESP32-S3-WROOM is a NeoPixel. An ‘intellegent’
device with its' own internal processor. But basically it accepts
commands to mix RGB values and display the resulting color. In this
sketch, we will treat the RGB NeoPixel as if it were actually three
LEDs (Red, Green, and Blue) in one package. If you drive the LEDs
digitally (On/Off) you get eight combinations (2^3); Black/Off, Red,
Green, Blue, Cyan, Magenta, Yellow, and White (all three on).

If you drive the assumed 3 LEDs using PWM (Pulse Width Modulation),
you can generate pseudo analog 256 levels from Off to On. You can
drive the the three LEDs at these different levels, 0 (Off) to 255
(full On) for all three RGB colors, to display a possible 16,777,216
(256^3) color combinations.
*/

/* The first user function eightColors() steps through all
 * eight of these colors. The function is called from loop()
 * and the actual function code is further down in the sketch.
 *
 * The eightColors() function turns the virtual individual LEDs
 * full-on or full-off. If you want to generate more than eight
 * colors, you can do so by varying the brightness level of the
 * individual sudo RGB LEDs between full-on, 255, and full-off, 0.
 *
 * Earlier sketches used the analogWrite() function to do this. This
 * function would let you dim a LED from full-off to full-on over
 * 255 steps.
 *
 * We will be using the neopixelWrite() function with our RGB LED,
 * which allows us to mix the RGB values digitally.
 *
 * The user function showSpectrum() smoothly steps through all
 * the colors. Again the function is called in loop() and the
 * actual code is defined further down in this sketch.
 */

//#define RGB_BUILTIN 48    // DO NOT REDEFINE RGB_BUILTIN
                            // If so, digitalWrite() will not work.

#define RGB_BRIGHTNESS 64   // Change White brightness (max 255)

int Red    = 128; // How much Red   0..255
int Green  = 128; // How much Green 0..255
int Blue   = 128; // How much Blue  0..255

int dDelay = 500; // delay/show time for the 8 Digital colors
int aDelay = 125; // delay between the blended RGB colors
int aStep  =  32; // the number of colors to skip per increment

void setup() {
  Serial.begin(115200);          // initialize Serial communication
  while(!Serial);                // wait for the Serial port to open

  #ifdef RGB_BUILTIN
  Serial.println("RGB_BUILTIN");
  #else
  Serial.println("no RGB_BUILTIN");
  #endif

  // No need to initialize the RGB LED!
}

void loop() {
  eightColors();    // display the eight digital colors
    delay(dDelay);
  showSpectrum();   // display the 16 MB analog colors
    delay(dDelay);
}

/* eightColors() -- displays the eight digital (Full-On/Off) colors
 * that the RGB LED can produce. The primary colors Red, Green, Blue;
 * are followed by the secondary 2-On colors Magenta, Green, Cyan;
 * White is all three colors On, and Black/Off is no-color.
 */
void eightColors() {
  Red    = 128; // How much Red   0..255, adjust for brightness.
  Green  = 128; // How much Green 0..255, adjust for brightness.
  Blue   = 128; // How much Blue  0..255, adjust for brightness.

  neopixelWrite(RGB_BUILTIN, 0, 0, 0);          // Black? (all Off - no color)
    delay(dDelay*4);
  neopixelWrite(RGB_BUILTIN, Red, 0, 0);        // Red
    delay(dDelay);
  neopixelWrite(RGB_BUILTIN, 0, Green, 0);      // Green
    delay(dDelay);
  neopixelWrite(RGB_BUILTIN, 0, 0, Blue);       // Blue
    delay(dDelay);
  neopixelWrite(RGB_BUILTIN, Red, 0, Blue);     // Magenta (Red and Blue)
    delay(dDelay);
  neopixelWrite(RGB_BUILTIN, Red, Green, 0);    // Yellow (Red and Green)
    delay(dDelay);
  neopixelWrite(RGB_BUILTIN, 0, Green, Blue);   // Cyan (Green and Blue)
    delay(dDelay);
  neopixelWrite(RGB_BUILTIN, Red, Green, Blue); // White (all three colors)
    delay(dDelay);
  neopixelWrite(RGB_BUILTIN, 0, 0, 0);          // Black? (all Off-no color)
    delay(dDelay);
}

/* showSpectrum() -- to generate a full spectrum of colors, we will
 * increment (via a step rate) through 16,777,216 possible colors;
 * 0..255 parts Red, Green and Blue. This function uses 3 nested
 * 'for' loops, and a defined step rate increment aStep.
 */
void showSpectrum() {
  for (Blue=0; Blue < 256; Blue += aStep) {
    for (Green=0; Green < 256; Green += aStep) {
      for (Red=0; Red < 256; Red += aStep) {
        neopixelWrite(RGB_BUILTIN, Red, Green, Blue);
        delay(aDelay);
      }
    }
  }
  neopixelWrite(RGB_BUILTIN, 0, 0, 0);  // Black? (all Off-no color)
  delay(dDelay);
}

====================================================================
```
