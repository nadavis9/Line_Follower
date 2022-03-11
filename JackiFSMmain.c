//*****************************************************************************
//
// Jacki FSM test main
// MSP432 with Jacki
// Daniel and Jonathan Valvano
// July 11, 2019
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

#include <stdint.h>
#include "msp.h"
#include "Clock.h"
#include "CortexM.h"
#include "PWM.h"
#include "LaunchPad.h"
#include "TExaS.h"
//#include "AP.h"
#include "../inc/UART0.h"
#include "Bump.h"
#include "Reflectance.h"
//#include "../inc/Motor.h"
#include "motor.h"
//#include "FlashProgram.c"

// Global Variables
uint8_t lineSense;
int32_t Position;
//Global Array for RAM dump and corresponding pointer
uint8_t debug_dump[512];
int debug_position = 0;
uint8_t line_sensors;
uint8_t  bump_sensors;
uint32_t  *ram_ptr = debug_dump;
uint32_t start_addr = 0x00020000;
#define START 0x00020000
int Semaphore;
int sys_tick_count = 0;
int ROM_count = 0;
uint32_t block_size = 0x00000200;
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
    int i = 0;
    for( i = 0; i < 512; i++){
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
    int i;
   for(i = 0; i < 4096; i++){
      curr_addr = ROM_start + 4*i;
      Flash_Write(curr_addr, 0xFFFFFFFF);
   }

}
//void Debug_FlashRecord(uint16_t *pt){
void Debug_FlashRecord(uint32_t *pt){
  // write this as part of Lab 10
   // uint32_t start_addr = 0x00020000;
    //uint32_t block_size = 0x00000200;
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
    int i;
    for( i = 0; i < 128; i++){
        //Flash_Write(start_addr + 4*i, *(pt + 4*i));
        uint32_t curr_addr = start_addr + 4*i;
        Flash_Write(curr_addr, *(pt+i));

    }
    //start_addr = start_addr + block_size;
}




/*void Motor_Select(uint32_t Output){

 switch (Output)
    {
    case 3:
        //PWM_Init(15000,500,500);
        Motor_Forward(5000,5000);
        break;
    case 2:
        //PWM_Init(15000,500,0);
        Motor_Right(2000,1000);
        //Motor_Forward(0,500);
        break;
    case 1:
        //PWM_Init(15000,0,500);
        Motor_Left(1000,2000);
        //Motor_Forward(0,500);
        break;
    case 0:
        //PWM_Init(15000,0,0);
        Motor_Stop();
        break;
    //default:
    //    Motor_Forward(100,100);
    }

}*/


//Struct for FSM




void main(void)
{
  Clock_Init48MHz();              // Initializing clock
  Reflectance_Init();             // Initializing line sensors
  //Bump_Init();                    // Initializing limit switches    Debug_Init();
  Debug_FlashInit();
  EdgeTrigger_Init();             //Initialize Bump Sensors to Edge Trigger
  SysTick_Init(48000,2);         // Set periodic interrupt to 1000Hz, priority 2
  Motor_Init();
  PWM_Init(15000,1000,1000);

  EnableInterrupts();

  uint16_t speed = 7000;
  uint8_t zero = 0;

  /*Run FSM continuously
  1) Output depends on State (LaunchPad LED)
  2) Wait depends on State
  3) Input (LaunchPad buttons)
  4) Next depends on (Input,State)
   */
  //Clock_Init48MHz();
  //LaunchPad_Init();
  //TExaS_Init(LOGICANALYZER);  // optional
  //Spt = Center;
  while(1){
    //WaitforInterrupt();
    //DutyA = motorDriver(Spt->out, Spt->delay);
    //Output = Spt->out;            // set output from FSM
    //Motor_Select(Output);         // do output to two motors
    //TExaS_Set(Input<<2|Output);   // optional, send data to logic analyzer
    //Clock_Delay1ms(Spt->delay);   // wait
    //Read line + bump sensors instead of LaunchPad

    //Input = LaunchPad_Input();    // read sensors
    //Input = lineSense;
    //Spt = Spt->next[Input];       // next depends on input and state

    if(Position == 117 | Position == 32| Position == 270)
    {
        Motor_Right(speed,speed);
        Clock_Delay1ms(200);
    }
    else if(Position == -279 | Position == -263| Position == -253)
    {
        Motor_Left(speed,speed);
        Clock_Delay1ms(200);
    }
    else if(Position == 0)
    {
        if(zero == 0)
        {
            Motor_Backward(2000,2000);
        }

        if(zero == 1)
        {
            Motor_Forward(speed,speed);
        }
    }
    else if(Position < -200)
    {
        Motor_Left(speed, speed);
        zero = 0;
    }
    else if(Position < -49 && Position > -201)
    {
        Motor_Forward(speed - 1000, 0);
        zero = 0;
    }
    else if (Position > 200)
    {
        Motor_Right(speed, speed);
        zero = 0;
    }
    else if(Position > 49 && Position < 201)
    {
        Motor_Forward(0,speed - 1000);
        zero = 0;
    }

    else
    {
        zero = 1;
        Motor_Forward(speed, speed);
    }
    if(Semaphore == 1)
    {
        Debug_FlashRecord(ram_ptr);
        start_addr = start_addr + block_size;

         ROM_count++;
              //P2->OUT &= ~0x01;
         Semaphore = 0;
         //ram_ptr = ram_ptr + 0x00000200;
    }
     if(ROM_count == 256)
    {
        return;
    }

  }


}

uint8_t intStart = 0;
uint8_t Data;

void SysTick_Handler(void)
{
    if(intStart == 0){
        Reflectance_Start();
    }
    else if(intStart == 1){
        //lineSense = Reflectance_End();           // Reads back line sensor data, 1 = sensor above line
        Data = Reflectance_End();
        Debug_Dump(Data, Bump_Read());
        sys_tick_count++;
    }

    if(sys_tick_count==256){
        Semaphore = 1;
        sys_tick_count = 0;
    }
    uint8_t bump_sensors = Bump_Read();
    //Debug_Dump(Data, bump_sensors);
    intStart = (intStart + 1) %5;
  Position = Reflectance_Position(Data);        // Calculates the position of the robot wrt the line, 332 (mm) to right of line, -332 (mm) to left of line
}
