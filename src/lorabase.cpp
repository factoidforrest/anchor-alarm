
#include <SPI.h>                                //the lora device is SPI based so load the SPI library                                         
#include <SX126XLT.h>                           //include the appropriate library  
#include <ArduinoJson.h>

SX126XLT LT;                                    //create a library class instance called LT

#define NSS 10                                  //select pin on LoRa device
#define NRESET 9                                //reset pin on LoRa device
#define RFBUSY 7                                //SX126X busy pin
#define DIO1 3                                  //DIO1 pin on LoRa device, used for sensing RX and TX done 
#define SW 5                                    //SW pin on LoRa device, used to power antenna switch
#define LORA_DEVICE DEVICE_SX1262               //we need to define the device we are using
#define TXpower 22                              //LoRa transmit power in dBm

uint8_t TXPacketL;
uint32_t TXPacketCount;


boolean setup_lora() {
  SPI.begin();

  if (!LT.begin(NSS, NRESET, RFBUSY, DIO1, SW, LORA_DEVICE)){
    return false;
  }

  LT.setupLoRa(915000000, 0, LORA_SF10, LORA_BW_020, LORA_CR_4_5, LDRO_ON); //configure frequency and LoRa settings
  LT.setSyncWord(0x1485);
  return true;
}




uint8_t TXPacketL;
void transmit_lora(int distance, int direction, int limit, boolean armed, boolean alarm) {

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
  LT.transmit(sendBufferInt, TXPacketL, 30000, TXpower, NO_WAIT)
  // set up to wait and watch for a 0 if you want to know if the transmit call failed

}

uint8_t RXBUFFER[256];

void receive_lora()
{
  int RXPacketL = LT.receive(RXBUFFER, 256, 300000, WAIT_RX); //wait for a packet to arrive with 5 min timeout

  if (RXPacketL == 0)                            //if the LT.receive() function detects an error, RXpacketL is 0
  {
    packet_is_Error();
  }
  else
  {
    parse_packet();
  }

}


struct ReceivedData {
  int distance;
  int direction;
  int limit;

  bool armed;
  bool alarm;
  int rssi;
  int snr;
  String error;
};

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
    snr
  };

  return received;
}

