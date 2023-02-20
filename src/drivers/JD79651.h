#ifndef LIB_JD79651
#define LIB_JD79651

#include "Adafruit_EPD.h"
#include <Arduino.h>

#define JD79651_DRIVER_CONTROL 0x01
#define JD79651_GATE_VOLTAGE 0x03
#define JD79651_SOURCE_VOLTAGE 0x04
#define JD79651_PROGOTP_INITIAL 0x08
#define JD79651_PROGREG_INITIAL 0x09
#define JD79651_READREG_INITIAL 0x0A
#define JD79651_BOOST_SOFTSTART 0x0C
#define JD79651_DEEP_SLEEP 0x10
#define JD79651_DATA_MODE 0x11
#define JD79651_SW_RESET 0x12
#define JD79651_TEMP_CONTROL 0x18
#define JD79651_TEMP_WRITE 0x1A
#define JD79651_MASTER_ACTIVATE 0x20
#define JD79651_DISP_CTRL1 0x21
#define JD79651_DISP_CTRL2 0x22
#define JD79651_WRITE_RAM1 0x24
#define JD79651_WRITE_RAM2 0x26
#define JD79651_WRITE_VCOM 0x2C
#define JD79651_READ_OTP 0x2D
#define JD79651_READ_STATUS 0x2F
#define JD79651_WRITE_LUT 0x32
#define JD79651_WRITE_BORDER 0x3C
#define JD79651_SET_RAMXPOS 0x44
#define JD79651_SET_RAMYPOS 0x45
#define JD79651_SET_RAMXCOUNT 0x4E
#define JD79651_SET_RAMYCOUNT 0x4F

/**************************************************************************/
/*!
    @brief  Class for interfacing with JD79651 EPD drivers
*/
/**************************************************************************/
class JD79651 : public Adafruit_EPD {
public:
  JD79651(int width, int height, int16_t SID, int16_t SCLK, int16_t DC,
                   int16_t RST, int16_t CS, int16_t SRCS, int16_t MISO,
                   int16_t BUSY = -1);
  JD79651(int width, int height, int16_t DC, int16_t RST, int16_t CS,
                   int16_t SRCS, int16_t BUSY = -1, SPIClass *spi = &SPI);

  void begin(bool reset = true);
  void priv_setup(); // hack!
  void set_buff(uint8_t * buff);
  void set_buff_memprog(uint8_t * buff);
  void private_display();
  void private_display2();
  void powerUp();
  void update(void);
  void display();  // hgiang: override
  void updatePartial(void);
  void powerDown();
  void displayPartial(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

protected:
  uint8_t writeRAMCommand(uint8_t index);
  void setRAMAddress(uint16_t x, uint16_t y);
  void setRAMWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
  void busy_wait();
};

#endif
