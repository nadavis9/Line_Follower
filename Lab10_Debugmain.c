// Lab10_Debugmain.c
// Runs on MSP432
// Student version to Debug lab
// Daniel and Jonathan Valvano
// September 4, 2017
// Interrupt interface for QTRX reflectance sensor array
// Pololu part number 3672.
// Debugging dump, and Flash black box recorder

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/
// Negative logic bump sensors
// P4.7 Bump5, left side of robot
// P4.6 Bump4
// P4.5 Bump3
// P4.3 Bump2
// P4.2 Bump1
// P4.0 Bump0, right side of robot

// reflectance even LED illuminate connected to P5.3
// reflectance odd LED illuminate connected to P9.2
// reflectance sensor 1 connected to P7.0 (robot's right, robot off road to left)
// reflectance sensor 2 connected to P7.1
// reflectance sensor 3 connected to P7.2
// reflectance sensor 4 connected to P7.3 center
// reflectance sensor 5 connected to P7.4 center
// reflectance sensor 6 connected to P7.5
// reflectance sensor 7 connected to P7.6
// reflectance sensor 8 connected to P7.7 (robot's left, robot off road to right)

#include "msp.h"
#include <stdint.h>
#include "..\inc\bump.h"
#include "..\inc\Reflectance.h"
#include "..\inc\Clock.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\CortexM.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\FlashProgram.h"


//Global Array for RAM dump and corresponding pointer
uint8_t debug_dump[512];
int debug_position = 0;
uint8_t line_sensors;
uint8_t  bump_sensors;
uint32_t  *ram_ptr = debug_dump;
//uint32_t start_addr = 0x00020000;
#define START 0x00020000
int Semaphore;
int sys_tick_count = 0;
int dump_count = 0;
//ram_ptr = (uint32_t volatile*)debug_dump;


void Debug_Init(void){
  // write this as part of Lab 10
  //initializes arrays
    //line_sensors = {'0','0','0','0','0','0','0','0'};
    //bump_sensors = {0,0,0,0,0,0,0,0};
    //for(int i = 0; i < 8; i++){
    //    line_sensors[i] = 0;
      //  bump_sensors[i] = 0;
    //}

    //malloc??????
    for(int i = 0; i < 512; i++){
        debug_dump[i] = 0;
    }

}
void Debug_Dump(uint8_t x, uint8_t y){
  // write this as part of Lab 10
  //saves data into RAM
    debug_dump[debug_position] = x;
    debug_dump[debug_position + 1] = y;
    debug_position = (debug_position + 2) % 512; //increment array position tracker, mod by 512 to stay within bounds of array


}
void Debug_FlashInit(void){ 
  // write this as part of Lab 10
  // erase 128 kibiB of flash ROM 0x00020000 to 0x0003FFFF
  // may pick block size from 32B to 512B (2^n is block size, 2^17/2^n is # of blocks
  //if data is 0xFF considered empty
  //BLOCK SIZE 512 = 2^9 bytes
  //2^17 /2^9 = 2^8 = 256 blocks of 512 bytes each
    uint32_t ROM_start = 0x00020000;
    uint32_t curr_addr = 0;/*
    uint32_t block_size = 0x00000200;
    uint32_t ROM_start = 0x00020000;
    uint32_t ROM_end = 0x0003FFF;
    for(int k = ROM_start,  k <ROM_end; k=k+4096){
        Flash_Erase(k);
    }*/
   // Flash_Init(Clock_GetFreq()/1000000);

   for(int i = 0; i < 4096; i++){
      curr_addr = ROM_start + 4*i;
      Flash_Write(curr_addr, 0xFFFFFFFF);
   }

}
//void Debug_FlashRecord(uint16_t *pt){
void Debug_FlashRecord(uint32_t *pt){
  // write this as part of Lab 10
   // uint32_t start_addr = 0x00020000;
    uint32_t block_size = 0x00000200;
    //Flash_Write(0x00020000, 0x12345678);
    //int write_count;
    //uint32_t curr_addr;
   // uint32_t curr_data;
  //will record 2^n bytes into next free block in flash ROM
    //Flash_Write(*pt, *ram_ptr);
    /*for(int i = 0; i < 512; i++){
        //curr_data = ((uint32_t)debug_dump[i]<<8
        Flash_WriteArray(*debug_dump, start_addr + (i*32), 1 );
    }*/
    for(int i = 0; i < 128; i++){
        //Flash_Write(start_addr + 4*i, *(pt + 4*i));
        uint32_t curr_addr = START + 4*i;
        Flash_Write(curr_addr, *(pt+i));

    }
    //start_addr = start_addr + block_size;
}
void SysTick_Handler(void){ // every 1ms
  // write this as part of Lab 10
  //call debug_dump 100 times per second
  //dump stores 2.56 seconds worth of data
    Debug_Dump(line_sensors, bump_sensors);
     /*if(Semaphore==1){
         Debug_FlashRecord(ram_ptr);
         dump_count++;
         P2->OUT &= ~0x01;
      Semaphore = 0;
     }*/
      sys_tick_count++;
      if(sys_tick_count==512){
          Semaphore = 1;
          sys_tick_count = 0;
      }


}

int main(void){
  // write this as part of Lab 10
   Clock_Init48MHz();
   SysTick_Init(6000000, 2); //call handler 100 times per second
   Debug_Init();
  //every 0.05*2^n seconds write 2^n bytes to flash RIM
  //stop after 655 seconds
  while(1){
     // bump_sensors = Bump_Read();
     // line_sensors = Reflectance_Read(1000);
      bump_sensors = 0x12; //1 2
      line_sensors = 0xAB; //10 11
      if(Semaphore==1){
          Debug_FlashRecord(ram_ptr);
          dump_count++;
          //P2->OUT &= ~0x01;
          Semaphore = 0;
      }
      if(dump_count == 256){
          return;
      }
  // write this as part of Lab 10

      }


}

int main11(void){ uint8_t data=0; //Program10_1
  Clock_Init48MHz();
  Debug_Init();
  LaunchPad_Init();
  while(1){
    P1->OUT |= 0x01;
    Debug_Dump(data,data+1);// linear sequence
    P1->OUT &= ~0x01;
    data=data+2;
  }
}


// Driver test
#define SIZE 256  // feel free to adjust the size
uint32_t Buffer[SIZE];
int main2(void){ //Program10_2
    uint16_t i;
  Clock_Init48MHz();
  LaunchPad_Init(); // built-in switches and LEDs
  for(i=0;i<SIZE;i++){
    Buffer[i] = (i<<8)+(255-i); // test data
  }
  i = 0;
  while(1){
    P1->OUT |= 0x01;
    Debug_FlashInit();
    P1->OUT &= ~0x01;
    P2->OUT |= 0x01;
    Debug_FlashRecord(Buffer); // 114us
    P2->OUT &= ~0x01;
    i++;
  }
}


int main3(void){
    uint16_t i;
  Clock_Init48MHz();
  LaunchPad_Init(); // built-in switches and LEDs
  //Flash_Init(Clock_GetFreq()/1000000);
  //Flash_Write(START + 0, 0x12345678);
  for(i=0;i<SIZE;i++){
    Buffer[i] = (i<<8)+(255-i); // test data
  }
  P1->OUT |= 0x01;

  P1->OUT &= ~0x01;
  i = 0;
  Debug_FlashInit();
  //Flash_Write(START + 0, 0x12345678);
  while(1){
    P2->OUT |= 0x01;
    Debug_FlashRecord(Buffer);
    P2->OUT &= ~0x01;
    i++;
  }
}

/*
uint8_t Buffer[1000];
uint32_t I=0;
uint8_t *pt;
void DumpI(uint8_t x){
  if(I<1000){
    Buffer[I]=x;
    I++;
  }
}
void DumpPt(uint8_t x){
  if(pt<&Buffer[1000]){
    *pt=x;
    pt++;
  }
}
void Activity(void){
  DumpI(5);
  DumpI(6);
  pt = Buffer;
  DumpPt(7);
  DumpPt(8);

}
*/
