 /** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2019 MAKERMAX INC.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of MAKERMAX INC. nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

// Include libraries this sketch will use
#include  <Wire.h>


void setup() {
  Wire.begin();

  Serial.begin(9600); // Open serial connection to send info to the host
  while (!Serial) {}  // wait for Serial comms to become ready
  Serial.println("Starting up");
  Serial.println("Testing device connection...");
  Wire.beginTransmission(0x4C);
  Wire.write(0x01);
  Wire.write(0x1F);
  Wire.endTransmission();

  pinMode(2,OUTPUT);
  pinMode(6,OUTPUT);
  
}

uint16_t level = 400; // Vary this from 0 - 4096 to control discharge current
uint16_t OFF=0;
uint16_t V1_regData=0;
float V1;

void loop() {


Wire.beginTransmission(0x4C);
Wire.write(0x02); //trigger
Wire.write(0x0); //trigger
Wire.endTransmission();
delay(200);

Wire.beginTransmission(0x4C);
Wire.write(0x07);
Wire.endTransmission();
Wire.requestFrom(0x4C,1);
byte LSB = Wire.read();

Wire.beginTransmission(0x4C);
Wire.write(0x06);
Wire.endTransmission();
Wire.requestFrom(0x4C,1);
byte MSB = Wire.read();
delay(100);

V1_regData = ((MSB << 8) | LSB) & ~0xC000;
V1 = (( V1_regData * 305.18) / 1000000);

Serial.println(LSB);
delay(100);
Serial.println(MSB);
delay(100);
Serial.println(V1_regData);
delay(500);
Serial.println(V1);

delay(1000);

}

float readV1()
{

float V1_regData;
float V1;
Wire.beginTransmission(0x4C);
Wire.write(0x02); //trigger
Wire.write(0x0); //trigger
Wire.endTransmission();
delay(200);

Wire.beginTransmission(0x4C);
Wire.write(0x07);
Wire.endTransmission();
Wire.requestFrom(0x4C,1);
byte LSB = Wire.read();

Wire.beginTransmission(0x4C);
Wire.write(0x06);
Wire.endTransmission();
Wire.requestFrom(0x4C,1);
byte MSB = Wire.read();
delay(100);

V1_regData = ((MSB << 8) | LSB) & ~0xC000;
V1 = (( V1_regData * 305.18) / 1000000);

return V1;

}


float readV2()
{

float V2_regData;
float V2;
Wire.beginTransmission(0x4C);
Wire.write(0x02); //trigger
Wire.write(0x0); //trigger
Wire.endTransmission();
delay(200);

Wire.beginTransmission(0x4C);
Wire.write(0x09);
Wire.endTransmission();
Wire.requestFrom(0x4C,1);
byte LSB = Wire.read();

Wire.beginTransmission(0x4C);
Wire.write(0x08);
Wire.endTransmission();
Wire.requestFrom(0x4C,1);
byte MSB = Wire.read();
delay(100);

V2_regData = ((MSB << 8) | LSB) & ~0xC000;
V2 = (( V2_regData * 305.18) / 1000000);

return V2;

}


float readV3()
{

float V3_regData;
float V3;
Wire.beginTransmission(0x4C);
Wire.write(0x02); //trigger
Wire.write(0x0); //trigger
Wire.endTransmission();
delay(200);

Wire.beginTransmission(0x4C);
Wire.write(0x0B);
Wire.endTransmission();
Wire.requestFrom(0x4C,1);
byte LSB = Wire.read();

Wire.beginTransmission(0x4C);
Wire.write(0x0A);
Wire.endTransmission();
Wire.requestFrom(0x4C,1);
byte MSB = Wire.read();
delay(100);

V3_regData = ((MSB << 8) | LSB) & ~0xC000;
V3 = (( V3_regData * 305.18) / 1000000);

return V3;

}

float readV4()
{

float V4_regData;
float V4;
Wire.beginTransmission(0x4C);
Wire.write(0x02); //trigger
Wire.write(0x0); //trigger
Wire.endTransmission();
delay(200);

Wire.beginTransmission(0x4C);
Wire.write(0x0D);
Wire.endTransmission();
Wire.requestFrom(0x4C,1);
byte LSB = Wire.read();

Wire.beginTransmission(0x4C);
Wire.write(0x0C);
Wire.endTransmission();
Wire.requestFrom(0x4C,1);
byte MSB = Wire.read();
delay(100);

V4_regData = ((MSB << 8) | LSB) & ~0xC000;
V4 = (( V4_regData * 305.18) / 1000000);

return V4;

}
