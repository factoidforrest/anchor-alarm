
#include <SPI.h>                                //the lora device is SPI based so load the SPI library                                         
#include <SX126XLT.h>                           //include the appropriate library  
#include <ArduinoJson.h>
#include <LoraFunctions.h>
#include <pins.h>
SX126XLT LT;                                    //create a library class instance called LT



// #define NSS 10                                  //select pin on LoRa device
// #define NRESET 9                                //reset pin on LoRa device
// #define RFBUSY 7                                //SX126X busy pin
// #define DIO1 3                                  //DIO1 pin on LoRa device, used for sensing RX and TX done 
// #define SW 5                                    //SW pin on LoRa device, used to power antenna switch
#define LORA_DEVICE DEVICE_SX1262               //we need to define the device we are using
#define TXpower 22                              //LoRa transmit power in dBm

uint8_t TXPacketL;
uint32_t TXPacketCount;


boolean setup_lora() {
  SPI.begin();

  if (!LT.begin(SS_LoRa, RST_LoRa_Dedupe, BUSY_LoRa_Dedupe, DIO1_LoRa, SW_LoRa, LORA_DEVICE)){
    return false;
  }

  // LT.setupLoRa(915000000, 0, LORA_SF11, LORA_BW_020, LORA_CR_4_5, LDRO_ON); //configure frequency and LoRa settings
  LT.setupLoRa(915000000, 0, LORA_SF11, LORA_BW_020, LORA_CR_4_5, LDRO_ON);
  // LT.setSyncWord(152);
  return true;
}




uint8_t transmit_lora(int distance, String direction, int limit, boolean armed, boolean alarm) {

  char sendBuffer[256]; 

  StaticJsonDocument<256> doc;
  doc["distance"] = distance;
  doc["direction"] = direction;
  doc["limit"] = limit;
  doc["armed"] = armed;
  doc["alarm"] = alarm;

  serializeJson(doc, sendBuffer);  // Serialize the JSON object into the buffer

  uint8_t* sendBufferInt = reinterpret_cast<uint8_t*>(sendBuffer);
  TXPacketL = strlen((char*)sendBuffer) + 1;  // Get the length of the JSON string
  uint8_t result = LT.transmit(sendBufferInt, TXPacketL, 50000, TXpower, WAIT_TX);
  return result;
  // set up to wait and watch for a 0 if you want to know if the transmit call failed

}

uint8_t RXBUFFER[255];


ReceivedData parse_packet()
{


  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, RXBUFFER);

  if (error) {
    ReceivedData errorStruct;
    errorStruct.error = error.f_str();
    return errorStruct;
  }

  int rssi = LT.readPacketRSSI();
  int snr = LT.readPacketSNR();

  ReceivedData received = {
    doc["distance"],
    doc["direction"],
    doc["limit"],
    doc["armed"],
    doc["alarm"],
    rssi,
    snr,
    ""
  };

  return received;
}


ReceivedData receive_lora()
{
  unsigned int packet_size = 255;
  int RXPacketL = LT.receive(RXBUFFER, packet_size, 50000, WAIT_RX); //wait for a packet to arrive with 1 min timeout

  Serial.print("PacketL:");
  Serial.println(RXPacketL);
  if (RXPacketL == 0)                            //if the LT.receive() function detects an error or timeout, RXpacketL is 0
  {
    ReceivedData errorStruct;
    LT.printDeviceErrors();
    errorStruct.error = "No Signal";
    return errorStruct;
  }
  else
  {
    return parse_packet();
  }

}


