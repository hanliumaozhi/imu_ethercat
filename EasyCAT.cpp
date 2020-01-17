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


//----- EasyCAT library for mbed boards -----------------------------------------------------
//----- Derived from the AB&T EasyCAT Arduino shield library V 1.5 170912--------------------------

//----- Tested with the STM32 NUCLEO-F767ZI board -------------------------------------------



#include "mbed.h"
#include "EasyCAT.h"                         


DigitalOut * SpiCs;								 	
SPI Spi(D11, D12, D13);                          // declare MOSI MISO SCK 


#define SCS_Low_macro  (*SpiCs = 0);			// macro for set/reset SPI chip select
#define SCS_High_macro (*SpiCs = 1);			//

												// macro for the SPI transfer		
inline static void SPI_TransferTx (unsigned char Data) 
{                                               //
	Spi.write(Data);                            //
};                                              //
                                                //
inline static void SPI_TransferTxLast (unsigned char Data) 
{                                               //
	Spi.write(Data);                            // 
};                                              //
                                                //
inline static unsigned char SPI_TransferRx (unsigned char Data) 
{                                               //
	return Spi.write(Data);                     //
};	                                            //


  
//---- constructors --------------------------------------------------------------------------------

EasyCAT::EasyCAT()                              //------- default constructor ---------------------- 
{                                               // 
  Sync_ = ASYNC;                                // if no synchronization mode is declared
                                                // ASYNC is the default
                                                //
  SCS_ = (PinName)D9;                           // if no chip select is declared                 
  SpiCs = new DigitalOut(SCS_, 1);              // pin D9 is the default  
}                                               //


EasyCAT::EasyCAT(PinName SCS) 					//------- SPI_CHIP_SELECT options -----------------
                                                //
                                                // for EasyCAT board REV_A we can choose between:
                                                // D8, D9, D10
                                                //  
                                                // for EasyCAT board REV_B we have three additional options:
                                                // A5, D6, D7 
{                                       
  SCS_ = SCS;                                   //  initialize chip select  
  SpiCs = new DigitalOut(SCS_, 1);				//
}                                               // 


EasyCAT::EasyCAT(SyncMode Sync)                 //-------Synchronization options ---------------------- 
                                                //   
                                                // we can choose between:
                                                // ASYNC   = free running i.e. no synchronization
                                                //           between master and slave (default)   
                                                //
                                                // DC_SYNC = interrupt is generated from the
                                                //           Distributed clock unit
                                                //
                                                // SM_SYNC = interrupt is generated from the
                                                //           Syncronizatiom manager 2 
{                                               //
  Sync_ = Sync;                                 //                                           
                                                //                                        
  SCS_ = (PinName)D9;                           // default chip select is pin 9 
  SpiCs = new DigitalOut(SCS_, 1);              //
}                                               //  


                                                //-- Synchronization and chip select options -----  
EasyCAT::EasyCAT(PinName SCS, SyncMode Sync) 	//
												//				
{                                               //
  Sync_ = Sync;                                 //  
                                                //   
  SCS_ = SCS;                                   //  initialize chip select                                                 
  SpiCs = new DigitalOut(SCS_, 1);              // 
}                                               //  


  
//---- EasyCAT board initialization ---------------------------------------------------------------


bool EasyCAT::Init()
{
  #define Tout 1000
  
  ULONG TempLong;
  unsigned short i;
   
  Spi.frequency(14000000);						          // SPI speed 14MHz  
  Spi.format(8,0);	  							          // 8 bit mode 0
  
  wait_ms(100);
 
  SPIWriteRegisterDirect (RESET_CTL, DIGITAL_RST);        // LAN9252 reset 
   
  i = 0;                                                  // reset timeout 
  do                                                      // wait for reset to complete
  {                                                       //
    i++;                                                  //
    TempLong.Long = SPIReadRegisterDirect (RESET_CTL, 4); //
  }while (((TempLong.Byte[0] & 0x01) != 0x00) && (i != Tout));    
                                                          //                                                       
  if (i == Tout)                                          // time out expired      
  {                                                       //   
    return false;                                         // initialization failed  
  }                                                         
  i = 0;                                                  // reset timeout  
  do                                                      // check the Byte Order Test Register
  {                                                       //
    i++;                                                  //      
    TempLong.Long = SPIReadRegisterDirect (BYTE_TEST, 4); //
  }while ((TempLong.Long != 0x87654321) && (i != Tout));  //                                                          
  if (i == Tout)                                          // time out expired      
  {                                                       // 
    return false;                                         // initialization failed  
  }            
  i = 0;                                                  // reset timeout  
  do                                                      // check the Ready flag
  {                                                       //
    i++;                                                  //    
    TempLong.Long = SPIReadRegisterDirect (HW_CFG, 4);    //
  }while (((TempLong.Byte[3] & READY) == 0) && (i != Tout));//
                                                          //
  if (i == Tout)                                          // time out expired      
  {                                                       //
    return false;                                         // initialization failed  
  } 

  
#ifdef BYTE_NUM
  printf ("STANDARD MODE\n"); 
#else
  printf ("CUSTOM MODE\n"); 
#endif

  printf ("%u Byte Out\n",TOT_BYTE_NUM_OUT);  
  printf ("%u Byte In\n",TOT_BYTE_NUM_IN);      

  printf ("Sync = ");                                                          
                                                            
                                                          
  if ((Sync_ == DC_SYNC) || (Sync_ == SM_SYNC))           //--- if requested, enable --------   
  {                                                       //--- interrupt generation -------- 
  
    if (Sync_ == DC_SYNC)
    {                                                     // enable interrupt from SYNC 0
      SPIWriteRegisterIndirect (0x00000004, AL_EVENT_MASK, 4);  
                                                          // in AL event mask register, and disable 
                                                          // other interrupt sources    
      printf("DC_SYNC\n");                                                      
    }                                                       
                                                                                                         
    else
    {                                                     // enable interrupt from SM 0 event
      SPIWriteRegisterIndirect (0x00000100, AL_EVENT_MASK, 4);  
                                                          // in AL event mask register, and disable 
                                                          // other interrupt sources 
      printf("SM_SYNC\n");    
    }   
                                                         
    SPIWriteRegisterDirect (IRQ_CFG, 0x00000111);         // set LAN9252 interrupt pin driver  
                                                          // as push-pull active high
                                                          // (On the EasyCAT shield board the IRQ pin
                                                          // is inverted by a mosfet, so Arduino                                                        
                                                          // receives an active low signal)
                                                                        
    SPIWriteRegisterDirect (INT_EN, 0x00000001);          // enable LAN9252 interrupt      
  } 

  else
  {
    printf("ASYNC\n");
  }
  TempLong.Long = SPIReadRegisterDirect (ID_REV, 4);      // read the chip identification 
  printf ("Detected chip ");                     		  // and revision, and print it
  printf ("%x ", TempLong.Word[1]);                       // out on the serial line
  printf (" Rev ");                                       //    
  printf ("%u \n", TempLong.Word[0]);                     //    
  
  
 /* 
  printf ("%u \n", TOT_BYTE_NUM_OUT); 
  printf ("%u \n", BYTE_NUM_OUT);                   
  printf ("%u \n", BYTE_NUM_ROUND_OUT);                
  printf ("%u \n", LONG_NUM_OUT);                  
     
  printf ("%u \n", SEC_BYTE_NUM_OUT);                    
  printf ("%u \n", SEC_BYTE_NUM_ROUND_OUT);                       
  printf ("%u \n\n", SEC_LONG_NUM_OUT);                   

  printf ("%u \n", TOT_BYTE_NUM_IN); 
  printf ("%u \n", BYTE_NUM_IN);                       
  printf ("%u \n", BYTE_NUM_ROUND_IN);                   
  printf ("%u \n", LONG_NUM_IN);                    
     
  printf ("%u \n", SEC_BYTE_NUM_IN);                    
  printf ("%u \n", SEC_BYTE_NUM_ROUND_IN);                         
  printf ("%u \n", SEC_LONG_NUM_IN);                         
*/
  
  
//--------------------------------------------------------------------------------------------  
  
  
  return true;                                            // initalization completed        
};  




//---- EtherCAT task ------------------------------------------------------------------------------

unsigned char EasyCAT::MainTask()                           // must be called cyclically by the application

{
  bool WatchDog = true;
  bool Operational = false; 
  unsigned char i;
  ULONG TempLong; 
  unsigned char Status;  
 
  
  TempLong.Long = SPIReadRegisterIndirect (WDOG_STATUS, 1); // read watchdog status
  if ((TempLong.Byte[0] & 0x01) == 0x01)                    //
    WatchDog = false;                                       // set/reset the corrisponding flag
  else                                                      //
    WatchDog = true;                                        //

    
  TempLong.Long = SPIReadRegisterIndirect (AL_STATUS, 1);   // read the EtherCAT State Machine status
  Status = TempLong.Byte[0] & 0x0F;                         //
  if (Status == ESM_OP)                                     // to see if we are in operational state
    Operational = true;                                     //
  else                                                      // set/reset the corrisponding flag
    Operational = false;                                    //    


                                                            //--- process data transfert ----------
                                                            //                                                        
  if (WatchDog | !Operational)                              // if watchdog is active or we are 
  {                                                         // not in operational state, reset 
    for (i=0; i < TOT_BYTE_NUM_OUT ; i++)                   // the output buffer
    BufferOut.Byte[i] = 0;                                  //

/*                                                          // debug
    if (!Operational)                                       //
      printf("Not operational\n");                    		//
    if (WatchDog)                                           //    
      printf("WatchDog\n");                           		//  
*/                                                          //
  }
  
  else                                                      
  {                                                         
    SPIReadProcRamFifo();                                   // otherwise transfer process data from 
  }                                                         // the EtherCAT core to the output buffer  
                 
  SPIWriteProcRamFifo();                                    // we always transfer process data from
                                                            // the input buffer to the EtherCAT core  

  if (WatchDog)                                             // return the status of the State Machine      
  {                                                         // and of the watchdog
    Status |= 0x80;                                         //
  }                                                         //
  return Status;                                            //   
  
}


    
//---- read a directly addressable registers  -----------------------------------------------------

unsigned long EasyCAT::SPIReadRegisterDirect (unsigned short Address, unsigned char Len)

                                                   // Address = register to read
                                                   // Len = number of bytes to read (1,2,3,4)
                                                   //
                                                   // a long is returned but only the requested bytes
                                                   // are meaningful, starting from LsByte                                                 
{
  ULONG Result; 
  UWORD Addr;
  Addr.Word = Address; 
  unsigned char i; 

  SCS_Low_macro                                             // SPI chip select enable

  SPI_TransferTx(COMM_SPI_READ);                            // SPI read command
  SPI_TransferTx(Addr.Byte[1]);                             // address of the register
  SPI_TransferTxLast(Addr.Byte[0]);                         // to read, MsByte first
 
  for (i=0; i<Len; i++)                                     // read the requested number of bytes
  {                                                         // LsByte first 
    Result.Byte[i] = SPI_TransferRx(DUMMY_BYTE);            //
  }                                                         //    
  
  SCS_High_macro                                            // SPI chip select disable 
 
  return Result.Long;                                       // return the result
}




//---- write a directly addressable registers  ----------------------------------------------------

void EasyCAT::SPIWriteRegisterDirect (unsigned short Address, unsigned long DataOut)

                                                   // Address = register to write
                                                   // DataOut = data to write
{ 
  ULONG Data; 
  UWORD Addr;
  Addr.Word = Address;
  Data.Long = DataOut;    

  SCS_Low_macro                                             // SPI chip select enable  
  
  SPI_TransferTx(COMM_SPI_WRITE);                           // SPI write command
  SPI_TransferTx(Addr.Byte[1]);                             // address of the register
  SPI_TransferTx(Addr.Byte[0]);                             // to write MsByte first

  SPI_TransferTx(Data.Byte[0]);                             // data to write 
  SPI_TransferTx(Data.Byte[1]);                             // LsByte first
  SPI_TransferTx(Data.Byte[2]);                             //
  SPI_TransferTxLast(Data.Byte[3]);                         //
 
  SCS_High_macro                                            // SPI chip select enable   
}


//---- read an undirectly addressable registers  --------------------------------------------------

unsigned long EasyCAT::SPIReadRegisterIndirect (unsigned short Address, unsigned char Len)

                                                   // Address = register to read
                                                   // Len = number of bytes to read (1,2,3,4)
                                                   //
                                                   // a long is returned but only the requested bytes
                                                   // are meaningful, starting from LsByte                                                  
{
  ULONG TempLong;
  UWORD Addr;
  Addr.Word = Address;
                                                            // compose the command
                                                            //
  TempLong.Byte[0] = Addr.Byte[0];                          // address of the register
  TempLong.Byte[1] = Addr.Byte[1];                          // to read, LsByte first
  TempLong.Byte[2] = Len;                                   // number of bytes to read
  TempLong.Byte[3] = ESC_READ;                              // ESC read 

  SPIWriteRegisterDirect (ECAT_CSR_CMD, TempLong.Long);     // write the command

  do
  {                                                         // wait for command execution
    TempLong.Long = SPIReadRegisterDirect(ECAT_CSR_CMD,4);  //
  }                                                         //
  while(TempLong.Byte[3] & ECAT_CSR_BUSY);                  //
                                                             
                                                              
  TempLong.Long = SPIReadRegisterDirect(ECAT_CSR_DATA,Len); // read the requested register
  return TempLong.Long;                                     //
}


//---- write an undirectly addressable registers  -------------------------------------------------

void  EasyCAT::SPIWriteRegisterIndirect (unsigned long DataOut, unsigned short Address, unsigned char Len)

                                                   // Address = register to write
                                                   // DataOut = data to write                                                    
{
  ULONG TempLong;
  UWORD Addr;
  Addr.Word = Address;


  SPIWriteRegisterDirect (ECAT_CSR_DATA, DataOut);            // write the data

                                                              // compose the command
                                                              //                                
  TempLong.Byte[0] = Addr.Byte[0];                            // address of the register  
  TempLong.Byte[1] = Addr.Byte[1];                            // to write, LsByte first
  TempLong.Byte[2] = Len;                                     // number of bytes to write
  TempLong.Byte[3] = ESC_WRITE;                               // ESC write

  SPIWriteRegisterDirect (ECAT_CSR_CMD, TempLong.Long);       // write the command

  do                                                          // wait for command execution
  {                                                           //
    TempLong.Long = SPIReadRegisterDirect (ECAT_CSR_CMD, 4);  //  
  }                                                           //  
  while (TempLong.Byte[3] & ECAT_CSR_BUSY);                   //
}


//---- read from process ram fifo ----------------------------------------------------------------


void EasyCAT::SPIReadProcRamFifo()    // read BYTE_NUM bytes from the output process ram, through the fifo
                                      //        
                                      // these are the bytes received from the EtherCAT master and
                                      // that will be use by our application to write the outputs
{
  ULONG TempLong;
  unsigned char i;
   
  #if TOT_BYTE_NUM_OUT > 0

    SPIWriteRegisterDirect (ECAT_PRAM_RD_CMD, PRAM_ABORT);        // abort any possible pending transfer

  
    SPIWriteRegisterDirect (ECAT_PRAM_RD_ADDR_LEN, (0x00001000 | (((uint32_t)TOT_BYTE_NUM_OUT) << 16)));   
                                                                  // the high word is the num of bytes
                                                                  // to read 0xTOT_BYTE_NUM_OUT----
                                                                  // the low word is the output process        
                                                                  // ram offset 0x----1000 

    SPIWriteRegisterDirect (ECAT_PRAM_RD_CMD, 0x80000000);        // start command        
 
 
                                                //------- one round is enough if we have ----
                                                //------- to transfer up to 64 bytes --------
   
    do                                                            // wait for the data to be       
    {                                                             // transferred from the output  
      TempLong.Long = SPIReadRegisterDirect (ECAT_PRAM_RD_CMD,2); // process ram to the read fifo       
    }                                                             //    
    while (TempLong.Byte[1] != FST_LONG_NUM_OUT);                 // 
  
    SCS_Low_macro                                                 // enable SPI chip select 
  
    SPI_TransferTx(COMM_SPI_READ);                                // SPI read command
    SPI_TransferTx(0x00);                                         // address of the read  
    SPI_TransferTxLast(0x00);                                     // fifo MsByte first
  
    for (i=0; i< FST_BYTE_NUM_ROUND_OUT; i++)                     // transfer the data
    {                                                             //
      BufferOut.Byte[i] = SPI_TransferRx(DUMMY_BYTE);             //
    }                                                             //
    
    SCS_High_macro                                                // disable SPI chip select    
  #endif  

  
  #if SEC_BYTE_NUM_OUT > 0                    //-- if we have to transfer more then 64 bytes --
                                              //-- we must do another round -------------------
                                              //-- to transfer the remainig bytes -------------


    do                                                          // wait for the data to be       
    {                                                           // transferred from the output  
      TempLong.Long = SPIReadRegisterDirect(ECAT_PRAM_RD_CMD,2);// process ram to the read fifo 
    }                                                           //    
    while (TempLong.Byte[1] != SEC_LONG_NUM_OUT);               //  

    SCS_Low_macro                                               // enable SPI chip select   
    
    SPI_TransferTx(COMM_SPI_READ);                              // SPI read command
    SPI_TransferTx(0x00);                                       // address of the read  
    SPI_TransferTxLast(0x00);                                   // fifo MsByte first
    
    for (i=0; i< (SEC_BYTE_NUM_ROUND_OUT); i++)                 // transfer loop for the remaining 
    {                                                           // bytes
      BufferOut.Byte[i+64] = SPI_TransferRx(DUMMY_BYTE);        // we transfer the second part of
    }                                                           // the buffer, so offset by 64
      
    SCS_High_macro                                              // SPI chip select disable  
  #endif    
}


//---- write to the process ram fifo --------------------------------------------------------------

void EasyCAT::SPIWriteProcRamFifo()    // write BYTE_NUM bytes to the input process ram, through the fifo
                                       //    
                                       // these are the bytes that we have read from the inputs of our                   
                                       // application and that will be sent to the EtherCAT master
{
  ULONG TempLong;
  unsigned char i;  
  
  
  
  #if TOT_BYTE_NUM_IN > 0  
  
    SPIWriteRegisterDirect (ECAT_PRAM_WR_CMD, PRAM_ABORT);        // abort any possible pending transfer
  
 
    SPIWriteRegisterDirect (ECAT_PRAM_WR_ADDR_LEN, (0x00001200 | (((uint32_t)TOT_BYTE_NUM_IN) << 16)));   
                                                                  // the high word is the num of bytes
                                                                  // to write 0xTOT_BYTE_NUM_IN----
                                                                  // the low word is the input process        
                                                                  // ram offset  0x----1200
                                                                                               
    SPIWriteRegisterDirect (ECAT_PRAM_WR_CMD, 0x80000000);        // start command  
  
  
                                                //------- one round is enough if we have ----
                                                //------- to transfer up to 64 bytes --------
    
    do                                                            // check that the fifo has      
    {                                                             // enough free space 
      TempLong.Long = SPIReadRegisterDirect (ECAT_PRAM_WR_CMD,2); //  
    }                                                             //  
    while (TempLong.Byte[1] < FST_LONG_NUM_IN);                   //
  
    SCS_Low_macro                                                 // enable SPI chip select
  
    SPI_TransferTx(COMM_SPI_WRITE);                               // SPI write command
    SPI_TransferTx(0x00);                                         // address of the write fifo 
    SPI_TransferTx(0x20);                                         // MsByte first 

    for (i=0; i< (FST_BYTE_NUM_ROUND_IN - 1 ); i++)               // transfer the data
    {                                                             //
      SPI_TransferTx (BufferIn.Byte[i]);                          // 
    }                                                             //
                                                                  //  
    SPI_TransferTxLast (BufferIn.Byte[i]);                        // one last byte
  
    SCS_High_macro                                                // disable SPI chip select           
  #endif        

  
  #if SEC_BYTE_NUM_IN > 0                     //-- if we have to transfer more then 64 bytes --
                                              //-- we must do another round -------------------
                                              //-- to transfer the remainig bytes -------------

    do                                                          // check that the fifo has     
    {                                                           // enough free space       
      TempLong.Long = SPIReadRegisterDirect(ECAT_PRAM_WR_CMD,2);// 
    }                                                           //  
    while (TempLong.Byte[1] < (SEC_LONG_NUM_IN));               //
                             
    SCS_Low_macro                                               // enable SPI chip select
    
    SPI_TransferTx(COMM_SPI_WRITE);                             // SPI write command
    SPI_TransferTx(0x00);                                       // address of the write fifo 
    SPI_TransferTx(0x20);                                       // MsByte first 

    for (i=0; i< (SEC_BYTE_NUM_ROUND_IN - 1); i++)              // transfer loop for the remaining 
    {                                                           // bytes
      SPI_TransferTx (BufferIn.Byte[i+64]);                     // we transfer the second part of
    }                                                           // the buffer, so offset by 64
                                                                //  
    SPI_TransferTxLast (BufferIn.Byte[i+64]);                   // one last byte  

    SCS_High_macro                                              // disable SPI chip select    
  #endif       
}

