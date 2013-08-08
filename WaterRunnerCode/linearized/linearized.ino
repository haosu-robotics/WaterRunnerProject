/*
  AnalogReadSerial
  Reads an analog input on pin 0, prints the result to the serial monitor.
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.
 
 This example code is in the public domain.
*/
#include <stdint.h>
#include "MatrixMath.h"

#define NUM_Y 2
#define NUM_U 4

uint32_t time = 0;
uint32_t lastTime = 0;
uint32_t lastSerialTime = 0;
uint32_t serialPer = 10000;
float dt;

// data types for sending and receiving bytes over serial
struct __attribute__ ((__packed__)) OutData
{
    float values[NUM_Y];
    uint8_t checksum;
};

union __attribute__ ((__packed__)) OutPacket
{
    OutData data;
    byte bytes[4*NUM_Y+1];
};

struct __attribute__ ((__packed__)) InData
{
    float values[NUM_U];  
    uint8_t checksum;
};

union __attribute__ ((__packed__))InPacket
{
    InData data;
    byte bytes[4*NUM_U+1];
};

OutPacket y;
InPacket u;
float ubak[NUM_U];

void sendData();
void receiveData();

//matrices for calculations
float xt0[5][1] =
{
    {0},
    {0},
    {0},
    {0},
    {0},
};

float xt1[5][1] =
{
    {0},
    {0},
    {0},
    {0},
    {0},
};

float A[5][5] =
{
    {0, 1, 0, 0, 0},
    {-147.9988, 0, 0, 0, -13.334},
    {0, 0, 0, 1, 0},
    {0, 0, -432.9137, 0, 0},
    {0, 0, 0, 0, 0},
};

float B[5][4] =
{
    {0, 0, 0, 0},
    {0.0906, 0.0906, 0.0466, 0.0466},
    {0, 0, 0, 0},
    {4.2406, -4.2406, 2.1815, -2.1815},
    {0, 0, 0, 0},
};

float Adt[5][5];
float Adtx[5][1]; 
float Bdt[5][4];
float Bdtu[5][1]; 
float dx[5][1]; 

// the setup routine runs once when you press reset:
void setup() {
    Serial.begin(115200);
    xt0[0][0] = 0.015;
    xt0[2][0] = 0.0;
    xt0[4][0] = 1;
    
    for(uint8_t i = 0; i < NUM_U; i++)
    {
        u.data.values[i] = 60;
    }

    y.data.values[0] = 0;
    y.data.values[1] = 0;
}

// the loop routine runs over and over again forever:
void loop() {
    lastTime = time;
    time = micros();
    dt = ((float) (time - lastTime))/1.e6;

    //A*dt*x
    Matrix.Copy((float*) A,5,5,(float *) Adt);
    Matrix.Scale((float *) Adt,5,5,dt);
    Matrix.Multiply((float*) Adt,(float*) xt0,5,5,1, (float*) Adtx);

    //B*dt*u
    Matrix.Copy((float*) B,5,4,(float *) Bdt);
    Matrix.Scale((float*) Bdt,5,4,dt);
    Matrix.Multiply((float*) Bdt,(float*) u.data.values,5,4,1, (float*) Bdtu);

    //A*dt*x + B*dt*u
    Matrix.Add((float*) Adtx,(float*) Bdtu,5,1,(float*) dx);

    //x_t = A*dt*x_{t-1} + B*dt*u + x_t-1
    Matrix.Add((float*) dx,(float*) xt0,5,1,(float*) xt1);
    Matrix.Copy((float*) xt1,5,1,(float*) xt0);

    if(Serial.available())
    {
        sendData();
        receiveData();
    }
}

void sendData()
{
    y.data.values[0] = xt0[0][0];
    y.data.values[1] = xt0[2][0];

    //compute checksum
    y.data.checksum = 0;
    for(uint8_t i = 0; i < 4*NUM_Y; i++)
    {
        y.data.checksum += y.bytes[i];
    }
    Serial.write(y.bytes,4*NUM_Y+1);
    return;
}

void receiveData()
{
    //create copy of old u incase checksum does not validate
    for(uint8_t i = 0; i < NUM_U; i++)
    {
        ubak[i] = u.data.values[i];
    }

    Serial.readBytes((char*) u.bytes,4*NUM_U+1);

    //check checksum
    uint8_t checksum = 0;
    for(uint8_t i = 0; i < 4*NUM_U; i++)
    {
        checksum += u.bytes[i];
    }

    //if checksum does not validate use last u
    if(checksum != u.data.checksum)
    {
        for(uint8_t i = 0; i < NUM_U; i++)
        {
            ubak[i] = u.data.values[i];
        }
    }
    return;
}
