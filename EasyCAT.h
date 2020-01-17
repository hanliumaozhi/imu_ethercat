//********************************************************************************************
//                                                                                           *
// AB&T Tecnologie Informatiche - Ivrea Italy                                                *
// http://www.bausano.net                                                                    *
// https://www.ethercat.org/en/products/791FFAA126AD43859920EA64384AD4FD.htm                 *
//                                                                                           *  
//********************************************************************************************    
//                                                                                           *
// This software is distributed as an example, in the hope that it could be useful,          *
// WITHOUT ANY WARRANTY, even the implied warranty of FITNESS FOR A PARTICULAR PURPOSE       *
//                                                                                           *
//******************************************************************************************** 


//----- EasyCAT library for mbed boards 170912 ----------------------------------------------
//----- Derived from the AB&T EasyCAT Arduino shield library V 1.5 --------------------------

//----- Tested with the STM32 NUCLEO-F767ZI board -------------------------------------------




#ifndef EasyCAT_H
#define EasyCAT_H

#define CUST_BYTE_NUM_OUT	1
#define CUST_BYTE_NUM_IN	40
#define TOT_BYTE_NUM_ROUND_OUT	4
#define TOT_BYTE_NUM_ROUND_IN	40



 
//****** PDO configuration parameters ******************************************************** 
 
// Here we can define how many I/O bytes our EasyCAT can exchange

// We can use STANDARD MODE or CUSTOM MODE
// !!! Warning: CUSTOM MODE is recommended only for advanced users !!!

// In STANDARD MODE the EasyCAT exchanges a fixed number of bytes 
// in input and in output
// The number of bytes in input are equal to the number of bytes in output
// Valid values are 16, 32, 64, 128  
//
// The configuration EEPROM on board of the EasyCAT must be  
// loaded with the corresponding bin file, provided by AB&T
//
// The EasyCAT is shipped configured for 32+32 bytes I.E. with
// the "EasyCAT_32_32.bin" file loaded into the configuration EEPROM  


//--   BYTE_NUM    -----------      Byte exchanged  ------    Config EEPROM file
                          
//#define BYTE_NUM   16         //      16 + 16               EasyCAT_16_16.bin 
//#define BYTE_NUM   32           //      32 + 32               EasyCAT_32_32.bin
//#define BYTE_NUM   64         //      64 + 64               EasyCAT_64_64.bin
//#define BYTE_NUM   128        //      128 + 128             EasyCAT_128_128.bin



// In CUSTOM MODE, recommended only for advanced users, the EasyCAT exchanges 
// a configurable number of bytes, between 0 and 128, in input and in output 
// The number of bytes in input and in output can be different
//
// The configuration EEPROM on board of the EasyCAT must be  
// loaded with a suitable bin file, provided by the user
// or with the "EasyCAT_configurable.bin" file provided by AB&T
 
// Comment all the above BYTE_NUM #define for the STANDARD MODE
// and define here the custom number of exchanged bytes 
 
                                           //----- CUSTOM_MODE example -------
                                           //         
//#define CUST_BYTE_NUM_OUT      49        // total number of bytes in output
//#define CUST_BYTE_NUM_IN       17        // total number of bytes in input

// We can also customize names and data types for the PDOs
// see the example in this file (look for tag "Custom data types example")
 
//*************************************************************************************
 
 
 
//-- the preprocessor calculates the parameters necessary to transfer out data --- 

                                                   // define TOT_BYTE_NUM_OUT as the total
                                                   // number of byte you need to
                                                   // transfer in output (between 0 and 128)
                                                   // this must match your ESI XML
                                                   
#ifdef  BYTE_NUM
  #define TOT_BYTE_NUM_OUT  BYTE_NUM                // bytes in output
#else  
  #define TOT_BYTE_NUM_OUT  CUST_BYTE_NUM_OUT       // any number between 0 and 128  
#endif


/*#if ((TOT_BYTE_NUM_OUT & 0x03) != 0x00)             // number of bytes in output
  #define TOT_BYTE_NUM_ROUND_OUT  ((TOT_BYTE_NUM_OUT | 0x03) + 1)  
#else                                               // rounded to 4 (long)
  #define TOT_BYTE_NUM_ROUND_OUT  TOT_BYTE_NUM_OUT  //
#endif                                              //
*/
#if TOT_BYTE_NUM_OUT > 64                           // if we have more then  64 bytes
                                                    // we have to split the transfer in two
                                                        
  #define SEC_BYTE_NUM_OUT  (TOT_BYTE_NUM_OUT - 64) // number of bytes of the second transfer
  
  #if ((SEC_BYTE_NUM_OUT & 0x03) != 0x00)           // number of bytes of the second transfer
    #define SEC_BYTE_NUM_ROUND_OUT  ((SEC_BYTE_NUM_OUT | 0x03) + 1)  
  #else                                             // rounded to 4 (long)
    #define SEC_BYTE_NUM_ROUND_OUT  SEC_BYTE_NUM_OUT//
  #endif                                            //

  #define SEC_LONG_NUM_OUT  SEC_BYTE_NUM_ROUND_OUT/4// number of long of the second transfer

  #define FST_BYTE_NUM_OUT  64                      // number of bytes of the first transfer     
  #define FST_BYTE_NUM_ROUND_OUT  64                // number of bytes of the first transfer
                                                    // rounded to 4 (long)
  #define FST_LONG_NUM_OUT  16                      // number of long of the second transfer

  
#else                                               // if we have 64 bytes max we tranfert
                                                    // them in just one round
                                                        
  #define FST_BYTE_NUM_OUT  TOT_BYTE_NUM_OUT        // number of bytes of the first and only transfer  

  #if ((FST_BYTE_NUM_OUT & 0x03) != 0x00)           // number of bytes of the first and only transfer
    #define FST_BYTE_NUM_ROUND_OUT ((FST_BYTE_NUM_OUT | 0x03) + 1)
  #else                                             // rounded to 4 (long)
    #define FST_BYTE_NUM_ROUND_OUT  FST_BYTE_NUM_OUT   
  #endif

  #define FST_LONG_NUM_OUT  FST_BYTE_NUM_ROUND_OUT/4// number of long of the first and only transfer
  
  #define SEC_BYTE_NUM_OUT  0                       // we don't use the second round
  #define SEC_BYTE_NUM_ROUND_OUT  0                 //
  #define SEC_LONG_NUM_OUT  0                       //
  
#endif


//-- the preprocessor calculates the parameters necessary to transfer in data --- 

                                                   // define TOT_BYTE_NUM_IN the total
                                                   // number of byte you need to
                                                   // transfer in input (between 0 and 128)
 
#ifdef  BYTE_NUM
  #define TOT_BYTE_NUM_IN  BYTE_NUM                // bytes in input
#else  
  #define TOT_BYTE_NUM_IN  CUST_BYTE_NUM_IN       // any number between 0 and 128  
#endif

    
/*#if ((TOT_BYTE_NUM_IN & 0x03) != 0x00)              // number of bytes in output
  #define TOT_BYTE_NUM_ROUND_IN  ((TOT_BYTE_NUM_IN | 0x03) + 1)  
#else                                               // rounded to 4 (long)
  #define TOT_BYTE_NUM_ROUND_IN  TOT_BYTE_NUM_IN    //
#endif                                              //
*/
#if TOT_BYTE_NUM_IN > 64                            // if we have more then  64 bytes
                                                    // we have to split the transfer in two
                                                        
  #define SEC_BYTE_NUM_IN  (TOT_BYTE_NUM_IN - 64)   // number of bytes of the second transfer
  
  #if ((SEC_BYTE_NUM_IN & 0x03) != 0x00)            // number of bytes of the second transfer
    #define SEC_BYTE_NUM_ROUND_IN  ((SEC_BYTE_NUM_IN | 0x03) + 1)  
  #else                                             // rounded to 4 (long)
    #define SEC_BYTE_NUM_ROUND_IN  SEC_BYTE_NUM_IN  //
  #endif                                            //

  #define SEC_LONG_NUM_IN  SEC_BYTE_NUM_ROUND_IN/4  // number of long of the second transfer

  #define FST_BYTE_NUM_IN  64                       // number of bytes of the first transfer     
  #define FST_BYTE_NUM_ROUND_IN  64                 // number of bytes of the first transfer
                                                    // rounded to 4 (long)
  #define FST_LONG_NUM_IN  16                       // number of long of the second transfer

  
#else                                               // if we have 64 bytes max we tranfert
                                                    // them in just one round
                                                        
  #define FST_BYTE_NUM_IN  TOT_BYTE_NUM_IN          // number of bytes of the first and only transfer  

  #if ((FST_BYTE_NUM_IN & 0x03) != 0x00)            // number of bytes of the first and only transfer
    #define FST_BYTE_NUM_ROUND_IN ((FST_BYTE_NUM_IN | 0x03) + 1)
  #else                                             // rounded to 4 (long)
    #define FST_BYTE_NUM_ROUND_IN  FST_BYTE_NUM_IN   
  #endif

  #define FST_LONG_NUM_IN  FST_BYTE_NUM_ROUND_IN/4  // number of long of the first and only transfer
  
  #define SEC_BYTE_NUM_IN  0                        // we don't use the second round
  #define SEC_BYTE_NUM_ROUND_IN  0                  //
  #define SEC_LONG_NUM_IN  0                        //

#endif
 
 
//---------------------------------------------------------------------------------

//----------------- sanity check -------------------------------------------------------                                 
      

#ifdef BYTE_NUM                     // STANDARD MODE and CUSTOM MODE
                                    // cannot be defined at the same time
  #ifdef CUST_BYTE_NUM_OUT 
    #error "BYTE_NUM and CUST_BYTE_NUM_OUT cannot be defined at the same time !!!!"
    #error "define them correctly in file EasyCAT.h"
    #endif
  
  #ifdef CUST_BYTE_NUM_IN 
    #error "BYTE_NUM and CUST_BYTE_NUM_IN cannot be defined at the same time !!!!"
    #error "define them correctly in file EasyCAT.h"
  #endif
#endif 
      
#ifdef BYTE_NUM                     //--- for BYTE_NUM we accept only 16  32  64  128 --
                                  
  #if ((BYTE_NUM !=16) && (BYTE_NUM !=32) && (BYTE_NUM !=64)  && (BYTE_NUM !=128))
    #error "BYTE_NUM must be 16, 32, 64 or 128 !!! define it correctly in file EasyCAT.h"
  #endif 
  
#else
                                   //--- CUST_BYTE_NUM_OUT and CUST_BYTE_NUM_IN --------
                                   //    must be max 128
  #if (CUST_BYTE_NUM_OUT > 128)
    #error "CUST_BYTE_NUM_OUT must be max 128 !!! define it correctly in file EasyCAT.h"
  #endif 
  
  #if (CUST_BYTE_NUM_IN > 128)
    #error "CUST_BYTE_NUM_IN must be max 128 !!! define it correctly in file EasyCAT.h"
  #endif 
  
#endif 

  
//*************************************************************************************************




//---- LAN9252 registers --------------------------------------------------------------------------

                                            //---- access to EtherCAT registers -------------------

#define ECAT_CSR_DATA           0x0300      // EtherCAT CSR Interface Data Register
#define ECAT_CSR_CMD            0x0304      // EtherCAT CSR Interface Command Register


                                            //---- access to EtherCAT process RAM ----------------- 

#define ECAT_PRAM_RD_ADDR_LEN   0x0308      // EtherCAT Process RAM Read Address and Length Register
#define ECAT_PRAM_RD_CMD        0x030C      // EtherCAT Process RAM Read Command Register
#define ECAT_PRAM_WR_ADDR_LEN   0x0310      // EtherCAT Process RAM Write Address and Length Register 
#define ECAT_PRAM_WR_CMD        0x0314      // EtherCAT Process RAM Write Command Register

#define ECAT_PRAM_RD_DATA       0x0000      // EtherCAT Process RAM Read Data FIFO
#define ECAT_PRAM_WR_DATA       0x0020      // EtherCAT Process RAM Write Data FIFO

                                            //---- EtherCAT registers -----------------------------
                                            
#define AL_STATUS               0x0130      // AL status
#define WDOG_STATUS             0x0440      // watch dog status
#define AL_EVENT_MASK           0x0204      // AL event interrupt mask

                                            //---- LAN9252 registers ------------------------------    

#define HW_CFG                  0x0074      // hardware configuration register
#define BYTE_TEST               0x0064      // byte order test register
#define RESET_CTL               0x01F8      // reset register       
#define ID_REV                  0x0050      // chip ID and revision
#define IRQ_CFG                 0x0054      // interrupt configuration
#define INT_EN                  0x005C      // interrupt enable


//---- LAN9252 flags ------------------------------------------------------------------------------

#define ECAT_CSR_BUSY     0x80

#define PRAM_ABORT        0x40000000

#define PRAM_BUSY         0x80

#define PRAM_AVAIL        0x01

#define READY             0x08

#define DIGITAL_RST       0x00000001


//---- EtherCAT flags -----------------------------------------------------------------------------

                                                      // EtherCAT state machine 
                                                      
#define ESM_INIT                0x01                  // init
#define ESM_PREOP               0x02                  // pre-operational
#define ESM_BOOT                0x03                  // bootstrap
#define ESM_SAFEOP              0x04                  // safe-operational
#define ESM_OP                  0x08                  // operational


//--- ESC commands --------------------------------------------------------------------------------

#define ESC_WRITE 		   0x80
#define ESC_READ 		     0xC0


//---- SPI ----------------------------------------------------------------------------------------

#define COMM_SPI_READ    0x03
#define COMM_SPI_WRITE   0x02

#define DUMMY_BYTE       0xFF



//---- typedef ------------------------------------------------------------------------------------

typedef union
{
    unsigned short  Word;
    unsigned char   Byte[2];
} UWORD;

typedef union
{
    unsigned long   Long;
    unsigned short  Word[2];
    unsigned char   Byte[4];
} ULONG;


typedef enum  
{
  ASYNC,
  DC_SYNC,
  SM_SYNC
}SyncMode;

typedef union												//---- output buffer ----
{
	uint8_t  Byte[TOT_BYTE_NUM_ROUND_OUT];
	struct
	{
		uint8_t     led_type;
	}Cust;
} PROCBUFFER_OUT;


typedef union												//---- input buffer ----
{
	uint8_t  Byte[TOT_BYTE_NUM_ROUND_IN];
	struct
	{
		float       acc_x;
		float       acc_y;
		float       acc_z;
		float       gyr_x;
		float       gyr_y;
		float       gyr_z;
		float       q0;
		float       q1;
		float       q2;
		float       q3;
	}Cust;
} PROCBUFFER_IN;

//-------------------------------------------------------------------------------------------------
 
class EasyCAT 
{
  public:
    EasyCAT();                              // default constructor
    EasyCAT(PinName SCS); 
    EasyCAT(SyncMode Sync);              
    EasyCAT(PinName SCS, SyncMode Sync);    
    
    
    unsigned char MainTask();               // EtherCAT main task
                                            // must be called cyclically by the application 
    
    bool Init();                            // EasyCAT board initialization 
    
    PROCBUFFER_OUT BufferOut;               // output process data buffer 
    PROCBUFFER_IN BufferIn;                 // input process data buffer
    
    
  private:
    void          SPIWriteRegisterDirect   (unsigned short Address, unsigned long DataOut);
    unsigned long SPIReadRegisterDirect    (unsigned short Address, unsigned char Len);
    
    void          SPIWriteRegisterIndirect (unsigned long  DataOut, unsigned short Address, unsigned char Len);
    unsigned long SPIReadRegisterIndirect  (unsigned short Address, unsigned char Len); 
    
    void          SPIReadProcRamFifo();    
    void          SPIWriteProcRamFifo();  
    
    PinName SCS_;     
    SyncMode Sync_;    
};

#endif

