#include <SPI.h>
#include "Arduino.h"
#include "util/crc16.h"
#include "SerialMessage.h"
#define STALL 1
#define _CS 10
#define UNSIGNE_LONG_MAX 4294967295

void readRegisters(uint8_t* registerAdresses, uint16_t *data16, size_t s) {
  if (s == 0) {
    return;
  }
  uint8_t* data8 = (uint8_t*)data16;
  digitalWrite(_CS, LOW);
//  SPI.transfer16(((uint16_t)(*registerAdresses++)) << 8);
  SPI.transfer(*registerAdresses++);
  SPI.transfer(0x00);
  digitalWrite(_CS, HIGH);
  while (--s > 0) {
    //delayMicroseconds(1);
    digitalWrite(_CS, LOW);
//    *data16++ = SPI.transfer16(((uint16_t)(*registerAdresses++)) << 8);
    *data8++ = SPI.transfer(*registerAdresses++);
    *data8++ = SPI.transfer(0x00);
    digitalWrite(_CS, HIGH);
  }
  //delayMicroseconds(1);
  digitalWrite(_CS, LOW);
  *data8++ = SPI.transfer(0x00);
  *data8++ = SPI.transfer(0x00);
//  *data16++ = SPI.transfer16(0x0000);
  digitalWrite(_CS, HIGH);

}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // Initialize serial output via USB
  SPI.begin();
  SPISettings settings(2000000, MSBFIRST, SPI_MODE3);
  SPI.beginTransaction(settings);
  pinMode(_CS, OUTPUT);
  digitalWrite(_CS, HIGH); //chip select
  //  pinMode(led, OUTPUT);
  uint8_t page = 0x0000;
  uint16_t trash;
  readRegisters(&page,&trash,1); //clk are low (firts transfer will fail)state after init of spi ater first write default is set to high 

  delay(500); // Give the part time to start up
  Serial.println(sizeof(long));
}


void loop() {
  Serial.setTimeout(UNSIGNE_LONG_MAX);
  Adis16488::Header header;
  if((Serial.readBytes((char*)&header, sizeof(Adis16488::Header)) == sizeof(Adis16488::Header)) && (header.start == SERIAL_HEADER_START_BYTE) ){
    uint8_t registerAddress[header.size];
    uint16_t registerData[header.size];
    Serial.setTimeout(100);
    uint8_t size = Serial.readBytes((char *)&registerAddress,header.size);
    if(size != header.size){
      Serial.flush();
      Serial.println("Data timeout.");  
      return;
      }
    Adis16488::Footer footer;
    if(!((Serial.readBytes((char *)&footer,sizeof(Adis16488::Footer))==sizeof(Adis16488::Footer)) && (footer.end == SERIAL_FOOTER_END_BYTE))){
      Serial.flush();
      Serial.println( "Bad footer");
      return;
      }
    readRegisters(registerAddress, registerData, header.size);
    Serial.write((uint8_t *)&registerData,header.size * 2);
  }
  else{
    Serial.flush();
    Serial.println("Timeout or bad header");  
  }
}
