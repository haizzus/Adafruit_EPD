#include "JD79651.h"
#include "Adafruit_EPD.h"
#include "Ap_29demo2.h"

#define EPD_RAM_BW 0x10
#define EPD_RAM_RED 0x13

#define BUSY_WAIT 500

/* /* this part copied from SAMPLE CODE ------------------------------------------*/
#include <SPI.h>
/* these are hacks, need to take the one pass from instance later */
int BUSY_Pin = A14; 
int RES_Pin = A15; 
int DC_Pin = A16; 
int CS_Pin = A17; 

#define EPD_W21_CS_0 digitalWrite(CS_Pin,LOW)
#define EPD_W21_CS_1 digitalWrite(CS_Pin,HIGH)
#define EPD_W21_DC_0  digitalWrite(DC_Pin,LOW)
#define EPD_W21_DC_1  digitalWrite(DC_Pin,HIGH)
#define EPD_W21_RST_0 digitalWrite(RES_Pin,LOW)
#define EPD_W21_RST_1 digitalWrite(RES_Pin,HIGH)
#define isEPD_W21_BUSY digitalRead(BUSY_Pin)

////////FUNCTION//////
void SPI_Write(unsigned char value);
void EPD_W21_WriteDATA(unsigned char command);
void EPD_W21_WriteCMD(unsigned char command);
//EPD
void EPD_W21_Init(void);
void EPD_init(void);
void EPD_sleep(void);
void EPD_refresh(void);
void lcd_chkstatus(void);
void PIC_display_Clean(void);
void PIC_display(const unsigned char* picData);

void __setup() {
   pinMode(BUSY_Pin, INPUT); 
   pinMode(RES_Pin, OUTPUT);  
   pinMode(DC_Pin, OUTPUT);    
   pinMode(CS_Pin, OUTPUT);    
   //SPI
   SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0)); 
   SPI.begin ();
}

//////////////////////SPI///////////////////////////////////

void SPI_Write(unsigned char value)                                    
{                                                           
    SPI.transfer(value);

}

void EPD_W21_WriteCMD(unsigned char command)
{
  EPD_W21_CS_0;                   
  EPD_W21_DC_0;   // command write
  SPI_Write(command);
  EPD_W21_CS_1;
}
void EPD_W21_WriteDATA(unsigned char command)
{
  EPD_W21_CS_0;                   
  EPD_W21_DC_1;   // command write
  SPI_Write(command);
  EPD_W21_CS_1;
}


/////////////////EPD settings Functions/////////////////////
void EPD_W21_Init(void)
{
  unsigned char i;
  for(i=0;i<3;i++) //Reset, three times to prevent the secondary electricity cannot be started
  {
    EPD_W21_RST_0;    // Module reset
    delay(10);//At least 10ms delay 
    EPD_W21_RST_1;
    delay(10);//At least 10ms delay 
  }
}
void EPD_init(void)
{
    EPD_W21_Init(); //Electronic paper IC reset
    
    EPD_W21_WriteCMD(0x04);  
    lcd_chkstatus();//waiting for the electronic paper IC to release the idle signal

    EPD_W21_WriteCMD(0x00);     //panel setting
    EPD_W21_WriteDATA(0x1f);    //LUT from OTP£¬KW-BF   KWR-AF  BWROTP 0f BWOTP 1f
  
    EPD_W21_WriteCMD(0X50);     //VCOM AND DATA INTERVAL SETTING      
    EPD_W21_WriteDATA(0x97);    //WBmode:VBDF 17|D7 VBDW 97 VBDB 57   WBRmode:VBDF F7 VBDW 77 VBDB 37  VBDR B7
}
void EPD_refresh(void)
{
    EPD_W21_WriteCMD(0x12);     //DISPLAY REFRESH   
    delay(10);          //!!!The delay here is necessary, 200uS at least!!!     
    lcd_chkstatus();
} 
void EPD_sleep(void)
{
    EPD_W21_WriteCMD(0X02);   //power off
    lcd_chkstatus();          //waiting for the electronic paper IC to release the idle signal
    //Part2 Increase the time delay
    delay(1000);      //Power off time delay, this is  necessary!!!     
    EPD_W21_WriteCMD(0X07);   //deep sleep
    EPD_W21_WriteDATA(0xA5);
}

void PIC_display(const unsigned char* picData)
{
    unsigned int i;
    EPD_W21_WriteCMD(0x10);        //Transfer old data
    for(i=0;i<2024;i++)      
    EPD_W21_WriteDATA(pgm_read_byte(&picData[i]));
  
    EPD_W21_WriteCMD(0x13);        //Transfer new data
    for(i=0;i<2024;i++)      
     EPD_W21_WriteDATA(pgm_read_byte(&picData[i]));

       //Refresh
    EPD_W21_WriteCMD(0x12);   //DISPLAY REFRESH   
    delay(10);    //!!!The delay here is necessary, 200uS at least!!!     
    lcd_chkstatus();          //waiting for the electronic paper IC to release the idle signal

}

void PIC_display_Clean(void)
{
    unsigned int i;
    EPD_W21_WriteCMD(0x10);        //Transfer old data
    for(i=0;i<2024;i++)       
  {
    EPD_W21_WriteDATA(0xff);
  }
  
    EPD_W21_WriteCMD(0x13);        //Transfer new data
    for(i=0;i<2024;i++)       
  {
    EPD_W21_WriteDATA(0xff);
  }
     //Refresh
    EPD_W21_WriteCMD(0x12);   //DISPLAY REFRESH   
    delay(10);    //!!!The delay here is necessary, 200uS at least!!!     
    lcd_chkstatus();          //waiting for the electronic paper IC to release the idle signal

}
void lcd_chkstatus(void)
{
  while(!isEPD_W21_BUSY);   //0:BUSY, 1:FREE                     
}


/* ------------------------------------------------------------------------------*/
// clang-format off

const uint8_t jd79651_default_init_code[] {
  JD79651_SW_RESET, 0, // soft reset
    0xFF, 20,          // busy wait
    JD79651_DATA_MODE, 1, 0x03, // Ram data entry mode
    JD79651_WRITE_BORDER, 1, 0x05, // border color
    JD79651_TEMP_CONTROL, 1, 0x80, // Temp control
    JD79651_SET_RAMXCOUNT, 1, 0,
    JD79651_SET_RAMYCOUNT, 2, 0, 0,
    0xFE};

// clang-format on

/**************************************************************************/
/*!
    @brief constructor if using external SRAM chip and software SPI
    @param width the width of the display in pixels
    @param height the height of the display in pixels
    @param SID the SID pin to use
    @param SCLK the SCLK pin to use
    @param DC the data/command pin to use
    @param RST the reset pin to use
    @param CS the chip select pin to use
    @param SRCS the SRAM chip select pin to use
    @param MISO the MISO pin to use
    @param BUSY the busy pin to use
*/
/**************************************************************************/
JD79651::JD79651(int width, int height, int16_t SID,
                                   int16_t SCLK, int16_t DC, int16_t RST,
                                   int16_t CS, int16_t SRCS, int16_t MISO,
                                   int16_t BUSY)
    : Adafruit_EPD(width, height, SID, SCLK, DC, RST, CS, SRCS, MISO, BUSY) {
  if ((height % 8) != 0) {
    height += 8 - (height % 8);
  }

  buffer1_size = ((uint32_t)width * (uint32_t)height) / 8;
  buffer2_size = buffer1_size;

  if (SRCS >= 0) {
    use_sram = true;
    buffer1_addr = 0;
    buffer2_addr = buffer1_size;
    buffer1 = buffer2 = NULL;
  } else {
    buffer1 = (uint8_t *)malloc(buffer1_size);
    buffer2 = (uint8_t *)malloc(buffer2_size);
  }

  singleByteTxns = true;
}

// constructor for hardware SPI - we indicate DataCommand, ChipSelect, Reset

/**************************************************************************/
/*!
    @brief constructor if using on-chip RAM and hardware SPI
    @param width the width of the display in pixels
    @param height the height of the display in pixels
    @param DC the data/command pin to use
    @param RST the reset pin to use
    @param CS the chip select pin to use
    @param SRCS the SRAM chip select pin to use
    @param BUSY the busy pin to use
*/
/**************************************************************************/
JD79651::JD79651(int width, int height, int16_t DC,
                                   int16_t RST, int16_t CS, int16_t SRCS,
                                   int16_t BUSY, SPIClass *spi)
    : Adafruit_EPD(width, height, DC, RST, CS, SRCS, BUSY, spi) {
  if ((height % 8) != 0) {
    height += 8 - (height % 8);
  }

  buffer1_size = ((uint32_t)width * (uint32_t)height) / 8;
  buffer2_size = buffer1_size;

  if (SRCS >= 0) {
    use_sram = true;
    buffer1_addr = 0;
    buffer2_addr = buffer1_size;
    buffer1 = buffer2 = NULL;
  } else {
    buffer1 = (uint8_t *)malloc(buffer1_size);
    buffer2 = (uint8_t *)malloc(buffer2_size);
  }

  singleByteTxns = true;
}

/**************************************************************************/
/*!
    @brief wait for busy signal to end
*/
/**************************************************************************/
void JD79651::busy_wait(void) {
  /*
  if (_busy_pin >= 0) {
    while (digitalRead(_busy_pin)) { // wait for busy low
      delay(10);
    }
  } else {
    delay(BUSY_WAIT);
  }
  */
 lcd_chkstatus();
}

/**************************************************************************/
/*!
    @brief begin communication with and set up the display.
    @param reset if true the reset pin will be toggled.
*/
/**************************************************************************/
void JD79651::begin(bool reset) {
  Adafruit_EPD::begin(reset);
  setBlackBuffer(0, true);  // black defaults to inverted
  setColorBuffer(1, false); // red defaults to un inverted
  powerDown();
}

void JD79651::priv_setup() {
  __setup();
  EPD_init();
}

void JD79651::set_buff(uint8_t * buff) {
  memcpy(buffer1, buff, buffer1_size);
}


void JD79651::set_buff_memprog(uint8_t * buff) {
  for(int i=0;i<buffer1_size;i++) 
    buffer1[i] = pgm_read_byte(&buff[i]);
}

void JD79651::private_display() {
      unsigned int i;
    //priv_setup();
    //EPD_init();


    const unsigned char* picData = gImage_1;
    //------------------PIC_display
    // EPD_W21_WriteCMD(0x10);        //Transfer old data
    for(i=0;i<2024;i++)      
     buffer1[i] = pgm_read_byte(&picData[i]);
  
    EPD_W21_WriteCMD(0x13);        //Transfer new data
    for(i=0;i<2024;i++)      
     EPD_W21_WriteDATA(buffer1[i]);

       //Refresh
    EPD_W21_WriteCMD(0x12);   //DISPLAY REFRESH   
    delay(10);    //!!!The delay here is necessary, 200uS at least!!!     
    lcd_chkstatus();          //waiting for the electronic paper IC to release the idle signal


   // ------------------------
    EPD_sleep();//EPD_sleep,Sleep instruction is necessary, please do not delete!!!

}
void JD79651::private_display2() {
  EPD_init(); //EPD init
  //PIC_display_Clean();
  memset(buffer1, 0xFF, buffer1_size);
  setCursor(20, 20);
  setTextColor(EPD_BLACK);
  setTextWrap(true);
  print("hello Massmelt!");


    EPD_W21_WriteCMD(0x13);        //Transfer new data
    for(int i=0;i<2024;i++)      
     EPD_W21_WriteDATA(buffer1[i]);

       //Refresh
    EPD_W21_WriteCMD(0x12);   //DISPLAY REFRESH   
    delay(10);    //!!!The delay here is necessary, 200uS at least!!!     
    lcd_chkstatus();    
}
/**************************************************************************/
/*!
    @brief signal the display to update
*/
/**************************************************************************/
void JD79651::update() {
  /*
  uint8_t buf[1];

  // display update sequence
  buf[0] = 0xF7;
  EPD_command(JD79651_DISP_CTRL2, buf, 1);

  EPD_command(JD79651_MASTER_ACTIVATE);
  busy_wait();

  if (_busy_pin <= -1) {
    delay(1000);
  }
  */
      //Refresh
    EPD_W21_WriteCMD(0x12);   //DISPLAY REFRESH   
    delay(10);    //!!!The delay here is necessary, 200uS at least!!!     
    lcd_chkstatus(); 

    default_refresh_delay = 1000;
}

void JD79651::display() {
    powerUp();
    // Set X & Y ram counters
    setRAMAddress(0, 0);

    EPD_W21_WriteCMD(0x13);        //Transfer new data


    writeRAMFramebufferToEPD(buffer1, buffer1_size, 0);
    if (buffer2_size != 0) {
    // oh there's another buffer eh?
    delay(2);

    // Set X & Y ram counters
    setRAMAddress(0, 0);

    if (use_sram) {
      writeSRAMFramebufferToEPD(buffer2_addr, buffer2_size, 1);
    } else {
      writeRAMFramebufferToEPD(buffer2, buffer2_size, 1);
    }
  }
    update();
  partialsSinceLastFullUpdate = 0;

//   if (sleep) {
// #ifdef EPD_DEBUG
//     Serial.println("  Powering Down");
// #endif
//     powerDown();
//   }
}
/**************************************************************************/
/*!
    @brief signal the display to update
*/
/**************************************************************************/
void JD79651::updatePartial(void) {
  uint8_t buf[1];

  // display update sequence
  buf[0] = 0xFF;
  EPD_command(JD79651_DISP_CTRL2, buf, 1);

  EPD_command(JD79651_MASTER_ACTIVATE);
  busy_wait();

  if (_busy_pin <= -1) {
    delay(1000);
  }
}

void JD79651::displayPartial(uint16_t x1, uint16_t y1, uint16_t x2,
                                      uint16_t y2) {
  // check rotation, move window around if necessary
  switch (getRotation()) {
  case 0:
    EPD_swap(x1, y1);
    EPD_swap(x2, y2);
    y1 = WIDTH - y1;
    y2 = WIDTH - y2;
    break;
  case 1:
    break;
  case 2:
    EPD_swap(x1, y1);
    EPD_swap(x2, y2);
    x1 = HEIGHT - x1;
    x2 = HEIGHT - x2;
    break;
  case 3:
    y1 = WIDTH - y1;
    y2 = WIDTH - y2;
    x1 = HEIGHT - x1;
    x2 = HEIGHT - x2;
  }
  if (x1 > x2)
    EPD_swap(x1, x2);
  if (y1 > y2)
    EPD_swap(y1, y2);

  /*
  Serial.print("x: ");
  Serial.print(x1);
  Serial.print(" -> ");
  Serial.println(x2);
  Serial.print("y: ");
  Serial.print(y1);
  Serial.print(" -> ");
  Serial.println(y2);
  */

  // x1 and x2 must be on byte boundaries
  x1 -= x1 % 8;           // round down;
  x2 = (x2 + 7) & ~0b111; // round up

  Serial.println("---------------partial=============");
  // perform standard power up
  powerUp();

  // display....
  // setRAMWindow(0, 0, 16/8, 16);
  setRAMWindow(x1 / 8, y1, x2 / 8, y2);
  setRAMAddress(x1 / 8, y1);

  // write image
  writeRAMCommand(0);

  Serial.print("Transfering: ");

  dcHigh();
  for (uint16_t y = y1; y < y2; y++) {
    for (uint16_t x = x1; x < x2; x += 8) {
      uint16_t i = (x / 8) + y * 25;
      SPItransfer(black_buffer[i]);
      // SPItransfer(0xAA);
    }
  }
  csHigh();

#ifdef EPD_DEBUG
  Serial.println("  UpdatePartial");
#endif

  updatePartial();

#ifdef EPD_DEBUG
  Serial.println("  partial Powering Down");
#endif

  powerDown();
}

/**************************************************************************/
/*!
    @brief start up the display
*/
/**************************************************************************/
void JD79651::powerUp() {
  uint8_t buf[5];

  hardwareReset();
  delay(100);
  busy_wait();
 /*
  const uint8_t *init_code = jd79651_default_init_code;

  if (_epd_init_code != NULL) {
    init_code = _epd_init_code;
  }
  EPD_commandList(init_code);

  // Set display size and driver output control
  buf[0] = (WIDTH - 1);
  buf[1] = (WIDTH - 1) >> 8;
  buf[2] = 0x00;
  EPD_command(JD79651_DRIVER_CONTROL, buf, 3);
*/
  EPD_init();
  setRAMWindow(0, 0, (HEIGHT / 8) - 1, WIDTH - 1);
}

/**************************************************************************/
/*!
    @brief wind down the display
*/
/**************************************************************************/
void JD79651::powerDown() {
  /*
  uint8_t buf[1];
  // Only deep sleep if we can get out of it
  if (_reset_pin >= 0) {
    // deep sleep
    buf[0] = 0x01;
    EPD_command(JD79651_DEEP_SLEEP, buf, 1);
    delay(100);
  } else {
    EPD_command(JD79651_SW_RESET);
    busy_wait();
  } */
  EPD_sleep();
}

/**************************************************************************/
/*!
    @brief Send the specific command to start writing to EPD display RAM
    @param index The index for which buffer to write (0 or 1 or tri-color
   displays) Ignored for monochrome displays.
    @returns The byte that is read from SPI at the same time as sending the
   command
*/
/**************************************************************************/
uint8_t JD79651::writeRAMCommand(uint8_t index) {
  if (index == 0) {
    return EPD_command(JD79651_WRITE_RAM1, false);
  }
  if (index == 1) {
    return EPD_command(JD79651_WRITE_RAM2, false);
  }
  return 0;
}

/**************************************************************************/
/*!
    @brief Some displays require setting the RAM address pointer
    @param x X address counter value
    @param y Y address counter value
*/
/**************************************************************************/
void JD79651::setRAMAddress(uint16_t x, uint16_t y) {
  uint8_t buf[2];

  // set RAM x address count
  buf[0] = x;
  EPD_command(JD79651_SET_RAMXCOUNT, buf, 1);

  // set RAM y address count
  buf[0] = y;
  buf[1] = y >> 8;
  EPD_command(JD79651_SET_RAMYCOUNT, buf, 2);
}

/**************************************************************************/
/*!
    @brief Some displays require setting the RAM address pointer
    @param x X address counter value
    @param y Y address counter value
*/
/**************************************************************************/
void JD79651::setRAMWindow(uint16_t x1, uint16_t y1, uint16_t x2,
                                    uint16_t y2) {
  uint8_t buf[5];

  // Set ram X start/end postion
  buf[0] = x1;
  buf[1] = x2;
  EPD_command(JD79651_SET_RAMXPOS, buf, 2);

  // Set ram Y start/end postion
  buf[0] = y1;
  buf[1] = y1 >> 8;
  buf[2] = y2;
  buf[3] = y2 >> 8;
  EPD_command(JD79651_SET_RAMYPOS, buf, 4);
}
