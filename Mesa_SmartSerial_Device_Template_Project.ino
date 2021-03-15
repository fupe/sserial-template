/*
* Mesa SmartSerial (SSLBP) device template project
*
* Copyright (C) 2020 Forest Darling <fdarling@gmail.com>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "LBP.h"
#include <stdarg.h>
#include <stdint.h>

// this code was based on the user fupeama's attachments on the following LinuxCNC forum post:
// https://forum.linuxcnc.org/27-driver-boards/34445-custom-board-for-smart-serial-interface?start=10#110007
// https://forum.linuxcnc.org/media/kunena/attachments/16679/sserial.h
// https://forum.linuxcnc.org/media/kunena/attachments/16679/sserial.c


#pragma pack(push,1)
int counter=0;
char buf[100];
int sw_mode=0;
int hw_mode=0;
uint8_t in[12];
/*static struct ProcessDataOut
{
  uint64_t input;
} pdata_out = {0x0000000000000000};

static struct ProcessDataIn
{
  uint32_t output;
} pdata_in = {0x0000000000000000};
*/

PROCESSDATA processdatain,processdataout;

static const char CARD_NAME[] = "cust"; // NOTE: LinuxCNC will force the
                                        // second character lowercase
                                        // stylize Mesa cards as 7i84
                                        // instead of 7I84, it's best
                                        // to use lowercase in the
                                        // first place...
static const uint32_t UNIT_NUMBER = 0x04030201;
static const uint16_t  GTOC_BASE_ADDRESS = 0x1000; // arbitrary, not real location in memory
static const uint16_t  PTOC_BASE_ADDRESS = 0x2000; // arbitrary, not real location in memory
static const uint16_t   PDD_BASE_ADDRESS = 0x3000; // arbitrary, not real location in memory
static const uint16_t PARAM_BASE_ADDRESS = 0x4000; // arbitrary, not real location in memory

static LBP_Discovery_Data DISCOVERY_DATA =
{
  .RxSize =0 ,//   = sizeof(ProcessDataOut)+1, // +1 for the fault status, remote transmits
  .TxSize =0 ,//   = sizeof(ProcessDataIn), // remote receives
  .ptoc   = PTOC_BASE_ADDRESS,
  .gtoc   = GTOC_BASE_ADDRESS
};


struct data_in_out
{
  uint32_t Output;   //index 4
  uint32_t Input;     //index 5
  uint16_t unsig2;
  uint16_t unsig3;
  uint16_t unsig4;
  uint16_t nvbaud;
  uint32_t nvunit;
  uint16_t hwrev;
}ddata;

static const LBP_PDD PDD[] =
{
  {.md = {.RecordType = LBP_PDD_RECORD_TYPE_MODE_DESCRIPTOR,.ModeIndex  = 0,.ModeType   = LBP_PDD_MODE_TYPE_HWMODE,._unused = 0,"Standard"}},   //index 0
  {.md = {.RecordType = LBP_PDD_RECORD_TYPE_MODE_DESCRIPTOR,.ModeIndex  = 0,.ModeType  = LBP_PDD_MODE_TYPE_SWMODE,._unused = 0,"Input_Output"} },//index 1
  {.md = {.RecordType = LBP_PDD_RECORD_TYPE_MODE_DESCRIPTOR,.ModeIndex  = 1,.ModeType   = LBP_PDD_MODE_TYPE_SWMODE,._unused = 0,"Input_Output2"}},//index 2
  {.md = {.RecordType = LBP_PDD_RECORD_TYPE_MODE_DESCRIPTOR,.ModeIndex  = 2,.ModeType   = LBP_PDD_MODE_TYPE_SWMODE,._unused = 0,"Input_Output3"}},//index 3
  {.pdd = {.RecordType  = LBP_PDD_RECORD_TYPE_NORMAL,.DataSize = 32,.DataType = LBP_PDD_DATA_TYPE_BITS,.DataDirection = LBP_PDD_DIRECTION_OUTPUT,.ParamMin = 0.0,.ParamMax = 0.0,.ParamAddress = PARAM_BASE_ADDRESS,"None\0Output\0\0"}},//index 4
  {.pdd = {.RecordType  = LBP_PDD_RECORD_TYPE_NORMAL,.DataSize = 32,.DataType = LBP_PDD_DATA_TYPE_BITS,.DataDirection = LBP_PDD_DIRECTION_INPUT, .ParamMin = 0.0,.ParamMax = 0.0,.ParamAddress = PARAM_BASE_ADDRESS+4,"None\0Input\0\0"}}, //index 5
  {.pdd = {.RecordType  = LBP_PDD_RECORD_TYPE_NORMAL,.DataSize = 16,.DataType = LBP_PDD_DATA_TYPE_UNSIGNED,.DataDirection = LBP_PDD_DIRECTION_INPUT, .ParamMin = 0.0,.ParamMax = 0x0,.ParamAddress = PARAM_BASE_ADDRESS+8,"None\0unsig2\0\0"}},//index 6
  {.pdd = {.RecordType  = LBP_PDD_RECORD_TYPE_NORMAL,.DataSize = 16,.DataType = LBP_PDD_DATA_TYPE_UNSIGNED,.DataDirection = LBP_PDD_DIRECTION_INPUT, .ParamMin = 0.0,.ParamMax = 0x0,.ParamAddress = PARAM_BASE_ADDRESS+10,"None\0unsig3\0\0"}},//index 7
  {.pdd = {.RecordType  = LBP_PDD_RECORD_TYPE_NORMAL,.DataSize = 16,.DataType = LBP_PDD_DATA_TYPE_UNSIGNED,.DataDirection = LBP_PDD_DIRECTION_INPUT, .ParamMin = 0.0,.ParamMax = 0x0,.ParamAddress = PARAM_BASE_ADDRESS+12,"None\0unsig4\0\0"}},//index 8
  {.pdd = {.RecordType  = LBP_PDD_RECORD_TYPE_NORMAL,.DataSize = 16,.DataType = LBP_PDD_DATA_TYPE_NONVOL_UNSIGNED,.DataDirection = LBP_PDD_DIRECTION_BI_DIRECTIONAL, .ParamMin = 0x0,.ParamMax = 0x0,.ParamAddress = PARAM_BASE_ADDRESS+14,"None\0NVBaudRate\0"}}, //9
  {.pdd = {.RecordType  = LBP_PDD_RECORD_TYPE_NORMAL,.DataSize = 32,.DataType = LBP_PDD_DATA_TYPE_NONVOL_UNSIGNED,.DataDirection = LBP_PDD_DIRECTION_INPUT, .ParamMin = 0x0,.ParamMax = 0x0,.ParamAddress = PARAM_BASE_ADDRESS+16,"None\0NVUnitNumber\0"}}, 
  {.pdd = {.RecordType  = LBP_PDD_RECORD_TYPE_NORMAL,.DataSize = 16,.DataType = LBP_PDD_DATA_TYPE_UNSIGNED,.DataDirection = LBP_PDD_DIRECTION_OUTPUT, .ParamMin = 0x0,.ParamMax = 0x0,.ParamAddress = PARAM_BASE_ADDRESS+20,"None\0HwRevision\0"}},
};


static const uint8_t PTOC_SEL[][12] =  //  12  because max bytes 96/8 =12
{
  {4,5},       //sw_mode=0
  {4,5,6},       //sw_mode=1
  {4,5,6,7,8}  //sw_mode=2
};

uint16_t PTOC[14];  // 12*8 bytes pdd + 2 bytes md
/*static const uint16_t PTOC[] =
{
  PDD_BASE_ADDRESS,
  PDD_BASE_ADDRESS+1*sizeof(LBP_PDD),
  PDD_BASE_ADDRESS+4*sizeof(LBP_PDD),
  PDD_BASE_ADDRESS+5*sizeof(LBP_PDD),
  PDD_BASE_ADDRESS+6*sizeof(LBP_PDD),
  0x0000
};*/
static const uint16_t GTOC[] =
{
  PDD_BASE_ADDRESS,
  PDD_BASE_ADDRESS+1*sizeof(LBP_PDD),
  PDD_BASE_ADDRESS+2*sizeof(LBP_PDD),
  PDD_BASE_ADDRESS+3*sizeof(LBP_PDD),
  PDD_BASE_ADDRESS+4*sizeof(LBP_PDD),
  PDD_BASE_ADDRESS+5*sizeof(LBP_PDD),
  PDD_BASE_ADDRESS+6*sizeof(LBP_PDD),
  PDD_BASE_ADDRESS+7*sizeof(LBP_PDD),
  PDD_BASE_ADDRESS+8*sizeof(LBP_PDD),
  PDD_BASE_ADDRESS+9*sizeof(LBP_PDD),
  PDD_BASE_ADDRESS+10*sizeof(LBP_PDD),
  PDD_BASE_ADDRESS+11*sizeof(LBP_PDD),
  0x0000
};
static const struct
{
  uint16_t base;
  uint16_t size;
  const void *data;
} VIRTUAL_MEMORY_MAP[] =
{
  {.base = GTOC_BASE_ADDRESS, .size = sizeof(GTOC), .data = GTOC},
  {.base = PTOC_BASE_ADDRESS, .size = sizeof(PTOC), .data = PTOC},
  {.base =  PDD_BASE_ADDRESS, .size = sizeof( PDD), .data = PDD},
  {.base =PARAM_BASE_ADDRESS, .size = sizeof(ddata), .data = &ddata},
};

#pragma pack(pop)

struct LBP_State
{
  uint16_t address;
} lbp_state =
{
  .address = 0x0000
};

//#define SHOW_DEBUG
//#define SHOW_VERBOSE
//#define SHOW_PDATA_IN
#define TEST

#ifdef SHOW_DEBUG
  #define DEBUG_PRINTF serialPrintf
  //#define DEBUG_PRINTF(f_, ...) do {Serial1.printf((f_), ##__VA_ARGS__);} while (0)
#else
  #define DEBUG_PRINTF(...)
#endif

#ifdef SHOW_VERBOSE
  #define VERB_PRINTF serialPrintf
#else
  #define VERB_PRINTF(...)   
#endif

#ifdef TEST
  #define TEST_PRINTF serialPrintf
#else
  #define TEST_PRINTF(...)   
#endif  

#if 0
#define SERIAL1_FLUSH Serial1.flush
#else
#define SERIAL1_FLUSH(x) do {} while (0)
#endif


#define SERIAL_PRINTF_MAX_BUFF      256
#define F_PRECISION                   6


void calculate_ptoc()
{


  //TEST_PRINTF("calcul sw_mode=%d \r\n",sw_mode);
  
   int in_bytes=0;
   int out_bytes=0;
   PTOC[0]=PDD_BASE_ADDRESS;
   PTOC[1]=PDD_BASE_ADDRESS+(1+sw_mode)*sizeof(LBP_PDD);
   for (size_t i = 0; i < 12; i++)
   {
    //TEST_PRINTF("i =%d  PTOC_SEL[%d][%d]]=%x\r\n",i,sw_mode,i,PTOC_SEL[sw_mode][i]);
   
    if (PTOC_SEL[sw_mode][i]!=0) 
    {
      //TEST_PRINTF("PTOC_SEL[sw_mode][i]=%d \r\n",PTOC_SEL[sw_mode][i]);
      PTOC[i+2]=PDD_BASE_ADDRESS+(PTOC_SEL[sw_mode][i])*sizeof(LBP_PDD);

      if (PDD[PTOC_SEL[sw_mode][i]].pdd.DataDirection==LBP_PDD_DIRECTION_INPUT)
      {
       in_bytes+=PDD[PTOC_SEL[sw_mode][i]].pdd.DataSize;
      }
      if (PDD[PTOC_SEL[sw_mode][i]].pdd.DataDirection==LBP_PDD_DIRECTION_OUTPUT)
      {
        out_bytes+=PDD[PTOC_SEL[sw_mode][i]].pdd.DataSize;
      }
   }
   else
   {
    PTOC[i+2]=0x0000;
    
   }
    
   DISCOVERY_DATA.RxSize=1+in_bytes/8;
   DISCOVERY_DATA.TxSize=out_bytes/8;
 }

/* TEST_PRINTF("DISCOVERY_DATA.RxSize =%d \r\n",DISCOVERY_DATA.RxSize);
 TEST_PRINTF("DISCOVERY_DATA.TxSize =%d \r\n",DISCOVERY_DATA.TxSize);
 for (int i=0;i<12;i++){ TEST_PRINTF("ptoc[%d] =0x%x    PDD[%d].pdd.ParamAddress=0x%x \r\n",i,PTOC[i],i,PDD[PTOC_SEL[sw_mode][i-2]].pdd.ParamAddress); if(PTOC[i]==0){      break;}}
 TEST_PRINTF(" \r\n");
  for (int i=0;i<=sizeof(PDD)/sizeof(PDD[0]);i++){TEST_PRINTF("gtoc[%d] =0x%x    PDD[%d].pdd.ParamAddress=0x%x \r\n",i,GTOC[i],i,(PDD[i].pdd.RecordType  == LBP_PDD_RECORD_TYPE_NORMAL) ? PDD[i].pdd.ParamAddress : 0 );}
*/

    
/*  TEST_PRINTF("---------------calkul------------------------\r\n");
  TEST_PRINTF("sw_mode =%d \r\n",sw_mode);
  TEST_PRINTF("hw_mode =%d \r\n",hw_mode);
  
  TEST_PRINTF("pdd_base =%d \r\n",PDD_BASE_ADDRESS);
  TEST_PRINTF("pdd_base2 =%d \r\n",(PDD_BASE_ADDRESS+1*sizeof(LBP_PDD)));*/
  
}
/**
 * --------------------------------------------------------------
 * Perform simple printing of formatted data
 * Supported conversion specifiers: 
 *      d, i     signed int
 *      u        unsigned int
 *      ld, li   signed long
 *      lu       unsigned long
 *      f        double
 *      s        string
 *      %        '%'
 * Usage: %[conversion specifier]
 * Note: This function does not support these format specifiers: 
 *      [flag][min width][precision][length modifier]
 * --------------------------------------------------------------
 */
void serialPrintf(const char *fmt, ...) {
  /* buffer for storing the formatted data */
  char buf[SERIAL_PRINTF_MAX_BUFF];
  char *pbuf = buf;
  /* pointer to the variable arguments list */
  va_list pargs;
  
  /* Initialise pargs to point to the first optional argument */
  va_start(pargs, fmt);
  /* Iterate through the formatted string to replace all 
  conversion specifiers with the respective values */
  while(*fmt) {
    if(*fmt == '%') {
      switch(*(++fmt)) {
        case 'd': 
        case 'i': 
          pbuf += sprintf(pbuf, "%d", va_arg(pargs, int));
          break;
        case 'x': 
          pbuf += sprintf(pbuf, "%x", va_arg(pargs, int));
          break;
        case 'u': 
          pbuf += sprintf(pbuf, "%u", va_arg(pargs, unsigned int));
          break;
        case 'l': 
          switch(*(++fmt)) {
            case 'd': 
            case 'i': 
              pbuf += sprintf(pbuf, "%ld", va_arg(pargs, long));
              break;
            case 'u': 
              pbuf += sprintf( pbuf, "%lu", 
                               va_arg(pargs, unsigned long));
              break;
          }
          break;
        case 'f': 
          pbuf += strlen(dtostrf( va_arg(pargs, double), 
                                  1, F_PRECISION, pbuf));
          break;
        
        case 'c':
          *(pbuf++) = (char)va_arg(pargs, int);
          break;
        case 's': 
          pbuf += sprintf(pbuf, "%s", va_arg(pargs, char *));
          break;
        case '%':
          *(pbuf++) = '%';
          break;
        default:
          break;
      }
    } else {
      *(pbuf++) = *fmt;
    }
    
    fmt++;
  }
  
  *pbuf = '\0';
  
  va_end(pargs);
  Serial.print(buf);
}

void SERIAL1_WRITE(const uint8_t *data, size_t len)
{
  for (size_t i = 0; i < len; i++)
  {
    VERB_PRINTF("Sending: 0x%X\r\n", static_cast<uint32_t>(data[i]));
  }
  Serial1.write(data, len);
}

void setup()
{
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, HIGH);
#ifdef TEST
   Serial.begin(115200); // baudrate doesn't matter, full speed USB always
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
#endif  
 
  
   TEST_PRINTF("-----------------------------------------\r\n");
   
   ddata.nvbaud=0x09;
   ddata.hwrev=0x08;
   ddata.nvunit=0x55331177;
  
  
 /* TEST_PRINTF("size of virt memory map=%x \r\n", sizeof(VIRTUAL_MEMORY_MAP));
  TEST_PRINTF("base gtoc[0]=0x%x \r\n",VIRTUAL_MEMORY_MAP[0].base);
  TEST_PRINTF("size gtoc[0]=0x%x  dec=%d \r\n",(VIRTUAL_MEMORY_MAP[0].size),(VIRTUAL_MEMORY_MAP[0].size));
  TEST_PRINTF("data gtoc[0]=%x \r\n",(VIRTUAL_MEMORY_MAP[0].data));
  TEST_PRINTF("\r\n");
  TEST_PRINTF("base ptoc[1]=0x%x \r\n",VIRTUAL_MEMORY_MAP[1].base);
  TEST_PRINTF("size ptoc[1]=%x  dec=%d \r\n",(VIRTUAL_MEMORY_MAP[1].size),(VIRTUAL_MEMORY_MAP[1].size));
  TEST_PRINTF("data ptoc[1]=%x \r\n",(VIRTUAL_MEMORY_MAP[1].data));
   TEST_PRINTF("\r\n");
  TEST_PRINTF("base pdd[2]=0x%x \r\n",VIRTUAL_MEMORY_MAP[2].base);
  TEST_PRINTF("size pdd[2]=%x   dec=%d\r\n",(VIRTUAL_MEMORY_MAP[2].size),(VIRTUAL_MEMORY_MAP[2].size));
  TEST_PRINTF("data pdd[2]=%x \r\n",(VIRTUAL_MEMORY_MAP[2].data));
   TEST_PRINTF("\r\n");
  TEST_PRINTF("base pdd[3]=0x%x \r\n",VIRTUAL_MEMORY_MAP[3].base);
  TEST_PRINTF("size pdd[3]=%x   dec=%d\r\n",(VIRTUAL_MEMORY_MAP[3].size),(VIRTUAL_MEMORY_MAP[3].size));
  TEST_PRINTF("data pdd[3]=%x \r\n",(VIRTUAL_MEMORY_MAP[3].data)); */
  
  sw_mode=1;
  TEST_PRINTF("sw_mode=%d \r\n",sw_mode);
 calculate_ptoc();
  TEST_PRINTF("DISCOVERY_DATA.RxSize =%d \r\n",DISCOVERY_DATA.RxSize);
 TEST_PRINTF("DISCOVERY_DATA.TxSize =%d \r\n",DISCOVERY_DATA.TxSize);
 for (int i=0;i<12;i++){ TEST_PRINTF("ptoc[%d] =0x%x    PDD[%d].pdd.ParamAddress=0x%x \r\n",i,PTOC[i],i,PDD[PTOC_SEL[sw_mode][i-2]].pdd.ParamAddress); if(PTOC[i]==0){      break;}}
 TEST_PRINTF(" \r\n");
  for (int i=0;i<=sizeof(PDD)/sizeof(PDD[0]);i++){TEST_PRINTF("gtoc[%d] =0x%x    PDD[%d].pdd.ParamAddress=0x%x \r\n",i,GTOC[i],i,(PDD[i].pdd.RecordType  == LBP_PDD_RECORD_TYPE_NORMAL) ? PDD[i].pdd.ParamAddress : 0 );}
  
  
  Serial1.begin(2500000); // 2.5MBps for Mesa Smart Serial
}

void loop()
{
  counter++;
  if (counter > 1000000)
  {
    counter=0;
    //TEST_PRINTF("processdatain[0]=%d \r\n",processdatain.uint8[0] );
  }

  
  digitalWrite(PC13, (millis() & 0x100) ? HIGH : LOW);
  if (Serial1.available())
  {
    LBP_Command cmd = {.value = static_cast<uint8_t>(Serial1.read())};
    //TEST_PRINTF("Received: %d\r\n", static_cast<uint32_t>(cmd.value));
    uint8_t crc = LBP_CalcNextCRC(cmd.value);
    if (cmd.Generic.CommandType == LBP_COMMAND_TYPE_READ_WRITE)
    {
      DEBUG_PRINTF("GOT %s COMMAND! (DataSize = %x, AddressSize = %x, AutoInc = %x, RPCIncludesData = %x)\r\n",
                   cmd.ReadWrite.Write ? "WRITE" : "READ",
                   static_cast<uint32_t>(1 << cmd.ReadWrite.DataSize),
                   static_cast<uint32_t>(cmd.ReadWrite.AddressSize),
                   static_cast<uint32_t>(cmd.ReadWrite.AutoInc),
                   static_cast<uint32_t>(cmd.ReadWrite.RPCIncludesData));

      // possibly read 2-byte address
      if (cmd.ReadWrite.AddressSize)
      {
        union
        {
          uint16_t address;
          uint8_t bytes[2];
        } addr;
        
        // read LSB
        while (!Serial1.available()) {yield();}
        addr.bytes[0] = Serial1.read();
        crc = LBP_CalcNextCRC(addr.bytes[0], crc);
        //TEST_PRINTF("Received LSB: 0x%x  ", static_cast<uint32_t>(addr.bytes[0]));

        // read MSB
        while (!Serial1.available()) {yield();}
        addr.bytes[1] = Serial1.read();
        crc = LBP_CalcNextCRC(addr.bytes[1], crc);
        //TEST_PRINTF(" MSB: 0x%x     ", static_cast<uint32_t>(addr.bytes[1]));

        lbp_state.address = addr.address;
        //TEST_PRINTF("lbp_state.address 0x%x\r\n", static_cast<uint32_t>(lbp_state.address));

        
      }
      
      if (cmd.ReadWrite.Write)
      {
        const uint8_t writeLength = 1 << cmd.ReadWrite.DataSize;
        TEST_PRINTF("readLength: %d\r\n", writeLength);
        
        TEST_PRINTF("Received LSB: 0x%x  \r\n", static_cast<uint32_t>(lbp_state.address));
        const void *src = NULL;
        
        for (size_t i = 0; i < sizeof(VIRTUAL_MEMORY_MAP)/sizeof(VIRTUAL_MEMORY_MAP[0]); i++)
        {


          if (i==3){TEST_PRINTF ("parameter base  4000  ");TEST_PRINTF("ar=%x \r\n",lbp_state.address);}
          if (lbp_state.address >= VIRTUAL_MEMORY_MAP[i].base && (lbp_state.address + writeLength) <= (VIRTUAL_MEMORY_MAP[i].base + VIRTUAL_MEMORY_MAP[i].size))
          {
            src = reinterpret_cast<const uint8_t*>(VIRTUAL_MEMORY_MAP[i].data) + (lbp_state.address - VIRTUAL_MEMORY_MAP[i].base);
          
           // uint8_t RESPONSE[sizeof(uint64_t)+1];
            //memcpy(RESPONSE, src, writeLength);
            //RESPONSE[writeLength] = LBP_CalcCRC(RESPONSE, writeLength);
            //TEST_PRINTF("R0=%x  \r\n",RESPONSE[0]);
            //TEST_PRINTF("R1=%x  \r\n",RESPONSE[1]);
            //TEST_PRINTF("R2=%x  \r\n",RESPONSE[2]);
            //TEST_PRINTF("R3=%x  \r\n",RESPONSE[3]);
            //TEST_PRINTF("nvunit=0x%x   addr nvunit =%x  \r\n",ddata.nvunit,&ddata.nvunit);
            ///TEST_PRINTF("crc=%x  \r\n",RESPONSE[2]);
            //uint8_t* aaa;
            //TEST_PRINTF(" ffff src=%x\r\n",src);
            //ddata.nvunit=0x11223344;
            //TEST_PRINTF("po nvunit=0x%x   addr nvunit =%x  \r\n",ddata.nvunit,&ddata.nvunit);
           // TEST_PRINTF("src=%x\r\n",src);
          //  RESPONSE[0]=0x22;
           // RESPONSE[1]=0x44;
           // RESPONSE[3]=0x66;
           // memcpy(const_cast<void*>(src), RESPONSE, writeLength);         
           // TEST_PRINTF("po po nvunit=0x%x   addr nvunit =%x  \r\n",ddata.nvunit,&ddata.nvunit);



//---------------
      uint8_t pdata_in_next[writeLength];
        for (size_t i = 0; i < writeLength; i++) // sizeof(pdata_in) == writeLength
        {
          while (!Serial1.available()) {yield();}
          const uint8_t c = Serial1.read();
          crc = LBP_CalcNextCRC(c, crc);
          pdata_in_next[i] = c;
        }
      while (!Serial1.available()) {yield();}
      const uint8_t lastByte = Serial1.read();
     // TEST_PRINTF("Received: 0x%x\r\n", static_cast<uint32_t>(lastByte));
      if (lastByte != crc)
      {
        DEBUG_PRINTF("<CRC bad>\r\n");
        return;
      }
      TEST_PRINTF("nvunit=0x%x   addr nvunit =%x  \r\n",ddata.nvunit,&ddata.nvunit);
      TEST_PRINTF("nvbaud=0x%x   addr nvunit =%x  \r\n",ddata.nvbaud,&ddata.nvbaud);
      memcpy(const_cast<void*>(src), pdata_in_next, writeLength);
      TEST_PRINTF("po po nvunit=0x%x   addr nvunit =%x  \r\n",ddata.nvunit,&ddata.nvunit);
      TEST_PRINTF("po po nvbaud=0x%x   addr nvunit =%x  \r\n",ddata.nvbaud,&ddata.nvbaud);
      const uint8_t RESPONSE[] = {LBP_CalcNextCRC(0x00)};
      SERIAL1_WRITE(RESPONSE, sizeof(RESPONSE));
      SERIAL1_FLUSH(); 
            //---------------------
            
            
            break;
          }
          
        }

        
         
      }
      else // (!cmd.ReadWrite.Write)
      {
        //DEBUG_PRINTF("specifically READ COMMAND: 0x%X\r\n", static_cast<uint32_t>(cmd.value));
        while (!Serial1.available()) {yield();}
        const uint8_t lastByte = Serial1.read();
        VERB_PRINTF("Received: 0x%X\r\n", static_cast<uint32_t>(lastByte));
        if (lastByte != crc)
        {
          DEBUG_PRINTF("<bad CRC>\r\n");
          return;
        }

        const uint8_t readLength = 1 << cmd.ReadWrite.DataSize;
        const void *src = NULL;
        for (size_t i = 0; i < sizeof(VIRTUAL_MEMORY_MAP)/sizeof(VIRTUAL_MEMORY_MAP[0]); i++)
        {


          if (i==3){TEST_PRINTF ("bacha 4000  ");TEST_PRINTF("ar=%x \r\n",lbp_state.address);
          TEST_PRINTF("i=%d  vir-base=0x%x  lenght=%i size=%d\r\n",i,VIRTUAL_MEMORY_MAP[i].base,readLength,VIRTUAL_MEMORY_MAP[i].size);}
          if (lbp_state.address >= VIRTUAL_MEMORY_MAP[i].base && (lbp_state.address + readLength) <= (VIRTUAL_MEMORY_MAP[i].base + VIRTUAL_MEMORY_MAP[i].size))
          {
            src = reinterpret_cast<const uint8_t*>(VIRTUAL_MEMORY_MAP[i].data) + (lbp_state.address - VIRTUAL_MEMORY_MAP[i].base);
            //TEST_PRINTF("ad=%x ",(VIRTUAL_MEMORY_MAP[i].data) + (lbp_state.address - VIRTUAL_MEMORY_MAP[i].base));
 //           TEST_PRINTF("ar=%x ",lbp_state.address);
            //TEST_PRINTF("VIRTUAL_MEMORY_MAP size %i=%x \r\n",i,VIRTUAL_MEMORY_MAP[i].size);
            //TEST_PRINTF("VIRTUAL_MEMORY_MAP data %i=%x \r\n",i,VIRTUAL_MEMORY_MAP[i].data);
            
            
            //TEST_PRINTF("src=%x\r\n",src);
            break;
          }
        }
        if (!src)
        {
          TEST_PRINTF("<invalid read address 0x%x>\r\n", static_cast<uint32_t>(lbp_state.address));
          //return;
        }
        uint8_t RESPONSE[sizeof(uint64_t)+1];
       // TEST_PRINTF("<sending %i bytes as response>\r\n", readLength);
        memcpy(RESPONSE, src, readLength);
        RESPONSE[readLength] = LBP_CalcCRC(RESPONSE, readLength);
        SERIAL1_WRITE(RESPONSE, readLength+1);
        SERIAL1_FLUSH();
      }
      
    }
    else if (cmd.Generic.CommandType == LBP_COMMAND_TYPE_RPC)
    {
      uint8_t pdata_in_next[DISCOVERY_DATA.TxSize];
      if (cmd.value == LBP_COMMAND_RPC_SMARTSERIAL_PROCESS_DATA)
      {
        for (size_t i = 0; i < DISCOVERY_DATA.TxSize; i++) // sizeof(pdata_in) == DISCOVERY_DATA.TxSize //xxx
        {
          while (!Serial1.available()) {yield();}
          const uint8_t c = Serial1.read();
          crc = LBP_CalcNextCRC(c, crc);
         //TEST_PRINTF("Received: 0x%x\r\n", static_cast<uint32_t>(c));
          pdata_in_next[i] = c;
        }
      }
      while (!Serial1.available()) {yield();}
      const uint8_t lastByte = Serial1.read();
     // TEST_PRINTF("Received: 0x%x\r\n", static_cast<uint32_t>(lastByte));
      if (lastByte != crc)
      {
        DEBUG_PRINTF("<CRC bad>\r\n");
        return;
      }
      switch (cmd.value)
      {
        case LBP_COMMAND_RPC_SMARTSERIAL_RPC_DISCOVERY:
        {
          VERB_PRINTF("got LBP_COMMAND_RPC_SMARTSERIAL_RPC_DISCOVERY\r\n");
          uint8_t RESPONSE[sizeof(DISCOVERY_DATA)+1];
          memcpy(RESPONSE, &DISCOVERY_DATA, sizeof(DISCOVERY_DATA));
          RESPONSE[sizeof(RESPONSE)-1] = LBP_CalcCRC(RESPONSE, sizeof(RESPONSE)-1);
          SERIAL1_WRITE(RESPONSE, sizeof(RESPONSE));
          SERIAL1_FLUSH();
          
          
        }
        break;
        
        case LBP_COMMAND_RPC_SMARTSERIAL_UNIT_NUMBER:
        {
          //TEST_PRINTF("got LBP_COMMAND_RPC_SMARTSERIAL_UNIT_NUMBER\r\n");
          uint8_t RESPONSE[sizeof(UNIT_NUMBER)+1];
          memcpy(RESPONSE, &UNIT_NUMBER, sizeof(UNIT_NUMBER));
          RESPONSE[sizeof(RESPONSE)-1] = LBP_CalcCRC(RESPONSE, sizeof(RESPONSE)-1);
          SERIAL1_WRITE(RESPONSE, sizeof(RESPONSE));
          SERIAL1_FLUSH();
          /*TEST_PRINTF("S-%x ",RESPONSE[0]);
          TEST_PRINTF("%x ",RESPONSE[1]);
          TEST_PRINTF(" %x ",RESPONSE[2]);
          TEST_PRINTF(" %x ",RESPONSE[3]);
          TEST_PRINTF("CRC %x \r\n",RESPONSE[4]);*/
          
          
        }
        break;

        case LBP_COMMAND_RPC_SMARTSERIAL_PROCESS_DATA:
        {
          VERB_PRINTF("got LBP_COMMAND_RPC_SMARTSERIAL_PROCESS_DATA\r\n");
          uint8_t RESPONSE[DISCOVERY_DATA.RxSize+1]; // +1 for CRC
          RESPONSE[0] = 0x00; // fault status
          //pdata_out.input = millis();
          //processdataout.uint8[0]=counter;
          //processdataout.uint8[1]=(static_cast<uint8_t>(millis()>>8));
          
          //pdata_out.input = 0b010101011010101001010101101010100101010110101010010101011010101001010101101010100101010110101010;
                            //000000001111111100000000111111110000000011111111000000001111111100000000111111110000000011111111
          //memcpy(RESPONSE+1, &pdata_out, sizeof(pdata_out)); // +1 for skipping fault status  //xxxx
          memcpy(RESPONSE+1, &processdataout, DISCOVERY_DATA.RxSize);
          RESPONSE[sizeof(RESPONSE)-1] = LBP_CalcCRC(RESPONSE, sizeof(RESPONSE)-1);
          SERIAL1_WRITE(RESPONSE, sizeof(RESPONSE));
          SERIAL1_FLUSH();

          memcpy(&processdatain, pdata_in_next, DISCOVERY_DATA.TxSize);
          //processdataout=processdatain;
#ifdef SHOW_PDATA_IN
          sprintf (buf, "P: 0x%08X, %i\r\n", pdata_in.output, Serial.available());
          Serial.println(buf);
#endif
          // TODO actually write outputs!

          // show the process data activity
          static uint8_t cnt = 0;
          cnt++;
          digitalWrite(PC13, (millis() & 0x100) ? HIGH : LOW);
          //processdataout.uint8[0]=ddata.nvbaud;
          //processdataout.uint8[0]=0b01010101;
        }
        break;
        
        default:
        DEBUG_PRINTF("   ***UNHANDLED*** LBP_COMMAND_TYPE_RPC: 0x%X\r\n", static_cast<uint32_t>(cmd.value));
      }
    }
    else if (cmd.Generic.CommandType == LBP_COMMAND_TYPE_LOCAL_READ_WRITE)
    {
      if (cmd.value >= 0xE0) // HACK check if it's a write command
      {
        //VERB_PRINTF("GOT LOCAL LBP WRITE COMMAND! 0x%X\r\n", static_cast<uint32_t>(cmd.value));
        uint8_t param = 0;
        if (cmd.value != LBP_COMMAND_LOCAL_WRITE_RESET_LBP_PARSE)
        {
          // skip parameter byte for now
          while (!Serial1.available()) {yield();}
          param = static_cast<uint8_t>(Serial1.read());
          //TEST_PRINTF("Received: 0x%x\r\n", static_cast<uint32_t>(param));
          crc = LBP_CalcNextCRC(param, crc);
        }
  
        while (!Serial1.available()) {yield();}
        const uint8_t lastByte = Serial1.read();
        VERB_PRINTF("Received: 0x%X\r\n", static_cast<uint32_t>(lastByte));
        if (lastByte != crc)
        {
          TEST_PRINTF("<CRC bad>    %x \r\n",lastByte);
         // return;   *********
        }
  
        // act
        switch (cmd.value)
        {
          case LBP_COMMAND_LOCAL_WRITE_STATUS:
          {
            //TEST_PRINTF("got LBP_COMMAND_LOCAL_WRITE_STATUS: 0x%x\r\n", static_cast<uint32_t>(param));
            const uint8_t RESPONSE[] = {LBP_CalcNextCRC(0x00)};
            SERIAL1_WRITE(RESPONSE, sizeof(RESPONSE));
            SERIAL1_FLUSH();
          }
          break;

          case LBP_COMMAND_LOCAL_WRITE_SW_MODE:
          {
           
            sw_mode=static_cast<uint32_t>(param);
            const uint8_t RESPONSE[] = {LBP_CalcNextCRC(0x00)};
            SERIAL1_WRITE(RESPONSE, sizeof(RESPONSE));
            SERIAL1_FLUSH();
            calculate_ptoc();
            /*for (int i=0; i<12; i++) {
            //TEST_PRINTF("PTOC_SEL[sw_mode][i]=%d \r\n",PTOC_SEL[sw_mode][i]);
            TEST_PRINTF("ptoc %d=%x \r\n",i,PTOC[i]);
            }*/
          }
          break;

          case LBP_COMMAND_LOCAL_WRITE_CLEAR_FAULTS:
          {
            VERB_PRINTF("got LBP_COMMAND_LOCAL_WRITE_CLEAR_FAULTS: 0x%X\r\n", static_cast<uint32_t>(param));
            const uint8_t RESPONSE[] = {LBP_CalcNextCRC(0x00)};
            SERIAL1_WRITE(RESPONSE, sizeof(RESPONSE));
            SERIAL1_FLUSH();
          }
          break;

          case LBP_COMMAND_LOCAL_WRITE_NVMEM_FLAG:
          {
            VERB_PRINTF("got LBP_COMMAND_LOCAL_WRITE_NVMEM_FLAG: 0x%X\r\n", static_cast<uint32_t>(param));
            const uint8_t RESPONSE[] = {LBP_CalcNextCRC(0x00)};
            SERIAL1_WRITE(RESPONSE, sizeof(RESPONSE));
            SERIAL1_FLUSH();
          }
          break;
          
          case LBP_COMMAND_LOCAL_WRITE_COMMAND_TIMEOUT:
          {
            //TEST_PRINTF("TIMEOUT: 0x%x\r\n", static_cast<uint32_t>(param));
            const uint8_t RESPONSE[] = {LBP_CalcNextCRC(0x00)};
            SERIAL1_WRITE(RESPONSE, sizeof(RESPONSE));
            SERIAL1_FLUSH();
          }
          break;
    
          default:
          DEBUG_PRINTF("   ***UNHANDLED*** LOCAL LBP WRITE COMMAND: 0x%X\r\n", static_cast<uint32_t>(cmd.value));
        }
      }
      else // if (cmd.value < 0xE0)
      {
        //VERB_PRINTF("GOT LOCAL LBP READ COMMAND! 0x%X\r\n", static_cast<uint32_t>(cmd.value));
        while (!Serial1.available()) {yield();}
        const uint8_t lastByte = Serial1.read();
        VERB_PRINTF("Received: 0x%X\r\n", static_cast<uint32_t>(lastByte));
        if (lastByte != crc)
        {
          TEST_PRINTF("<CRC bad>  <e0 \r\n");
          return;
        }
  
        // respond
        switch (cmd.value)
        {
          case LBP_COMMAND_LOCAL_READ_LBP_STATUS:
          {
            TEST_PRINTF("LBP_S\r\n");
            const uint8_t lbp_status = 0x00;
            const uint8_t RESPONSE[] = {lbp_status, LBP_CalcNextCRC(lbp_status)};
            SERIAL1_WRITE(RESPONSE, sizeof(RESPONSE));
            SERIAL1_FLUSH();
          }
          break;

          case LBP_COMMAND_LOCAL_READ_CLEAR_FAULT_FLAG:
          {
            VERB_PRINTF("got LBP_COMMAND_LOCAL_READ_CLEAR_FAULT_FLAG\r\n");
            const uint8_t fault_flag = 0x00;
            const uint8_t RESPONSE[] = {fault_flag, LBP_CalcNextCRC(fault_flag)};
            SERIAL1_WRITE(RESPONSE, sizeof(RESPONSE));
            SERIAL1_FLUSH();
          }
          break;

          case LBP_COMMAND_LOCAL_READ_CARD_NAME_CHAR0:
          case LBP_COMMAND_LOCAL_READ_CARD_NAME_CHAR1:
          case LBP_COMMAND_LOCAL_READ_CARD_NAME_CHAR2:
          case LBP_COMMAND_LOCAL_READ_CARD_NAME_CHAR3:
          {
            //TEST_PRINTF("got LBP_COMMAND_LOCAL_READ_CARD_NAME_CHAR%i\r\n", static_cast<uint32_t>(cmd.value - LBP_COMMAND_LOCAL_READ_CARD_NAME_CHAR0));
            uint8_t RESPONSE[] = {CARD_NAME[cmd.value - LBP_COMMAND_LOCAL_READ_CARD_NAME_CHAR0], 0x00};
            RESPONSE[sizeof(RESPONSE)-1] = LBP_CalcCRC(RESPONSE, sizeof(RESPONSE)-1);
            SERIAL1_WRITE(RESPONSE, sizeof(RESPONSE));
            SERIAL1_FLUSH();
          }
          break;

          case LBP_COMMAND_LOCAL_READ_FAULT_DATA:
          {
            VERB_PRINTF("got LBP_COMMAND_LOCAL_READ_FAULT_DATA\r\n");
            const uint8_t fault_data = 0x00;
            const uint8_t RESPONSE[] = {fault_data, LBP_CalcNextCRC(fault_data)};
            SERIAL1_WRITE(RESPONSE, sizeof(RESPONSE));
            SERIAL1_FLUSH();
          }
          break;
          
          case LBP_COMMAND_LOCAL_READ_COOKIE:
          {
            TEST_PRINTF("COOKIE\r\n");
            const uint8_t RESPONSE[] = {LBP_COOKIE, LBP_CalcNextCRC(LBP_COOKIE)};
            SERIAL1_WRITE(RESPONSE, sizeof(RESPONSE));
            SERIAL1_FLUSH();
            
          }
          break;
    
          default:
          DEBUG_PRINTF("   ***UNHANDLED*** LOCAL LBP READ COMMAND: 0x%X\r\n", static_cast<uint32_t>(cmd.value));
        }
      }
    }
    else
    {
      DEBUG_PRINTF("unknown command %02X\r\n", static_cast<uint32_t>(cmd.value));
    }
  }
}
