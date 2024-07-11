/*
   rocrail_CAN_TCP_GW

    Pogramme permettant le pilotage de locomotives à partir de Rocrail©
    et la mise a jour de la liste des locomotives en MFX en utilisant une liaison TCP (WiFi ou Ethernet).


*/

#define PROJECT "rocrail_can_tcp_gateway"
#define VERSION "0.3"
#define AUTHOR "Christophe BOBILLE"

//----------------------------------------------------------------------------------------
//  Board Check
//----------------------------------------------------------------------------------------

#ifndef ARDUINO_ARCH_ESP32
#error "Select an ESP32 board"
#endif


//----------------------------------------------------------------------------------------
//   Include files
//----------------------------------------------------------------------------------------

#include <ACAN_ESP32.h> // https://github.com/pierremolinaro/acan-esp32.git
#include <Arduino.h>

//----------------------------------------------------------------------------------------
//   Debug serial
//----------------------------------------------------------------------------------------

#define RXD1 13
#define TXD1 14

#define debug Serial // (only for TCP connection)
// #define debug Serial1 // (only for USB connection)

//----------------------------------------------------------------------------------------
//  CAN Desired Bit Rate
//----------------------------------------------------------------------------------------

static const uint32_t DESIRED_BIT_RATE = 250UL * 1000UL;  // Marklin CAN baudrate = 250Kbit/s

//----------------------------------------------------------------------------------------
//  Buffers  : Rocrail always send 13 bytes
//----------------------------------------------------------------------------------------

byte cBuffer[13]; // CAN buffer
byte sBuffer[13]; // Serial buffer

//----------------------------------------------------------------------------------------
//  Marklin hash
//----------------------------------------------------------------------------------------

uint16_t RrHash; // for Rocrail hash

//----------------------------------------------------------------------------------------
//  TCP/WIFI
//----------------------------------------------------------------------------------------

#include <WiFi.h>
const char *ssid = "*************";
const char *password = "*************";
const uint port = 15731;
const char *ip = "192.168.1.13"; // tcpbin.com's ip
WiFiServer server(port);
WiFiClient client;

//----------------------------------------------------------------------------------------
//  Debug declaration
//----------------------------------------------------------------------------------------

void debugFrame(const CANMessage*);

//----------------------------------------------------------------------------------------
//   SETUP
//----------------------------------------------------------------------------------------

void setup()

{
  //--- Start serial
  // (only for USB connection)
  // Serial.begin(500000); // Rocrail baudrate to ESP32 USB
  // delay(100);
  // Serial.setTimeout(2);
  // debug.begin(115200, SERIAL_8N1, RXD1, TXD1); // For debug

  //5only for tcp connection)
  debug.begin(115200);

  while (!Serial)
  {
    debug.print(".");
    delay(200);
  }
  delay(100);

  debug.println("Start setup");

  //--- Configure ESP32 CAN

  debug.println("Configure ESP32 CAN");
  ACAN_ESP32_Settings settings(DESIRED_BIT_RATE);
  settings.mRxPin = GPIO_NUM_22; // Optional, default Tx pin is GPIO_NUM_4
  settings.mTxPin = GPIO_NUM_23; // Optional, default Rx pin is GPIO_NUM_5
  const uint32_t errorCode = ACAN_ESP32::can.begin(settings);

  if (errorCode)
  {
    debug.print("Configuration error 0x");
    debug.println(errorCode, HEX);
  }
  else
    debug.println("Configuration CAN OK");

  debug.println("");

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  server.begin();

  while (!client)   // listen for incoming clients
    client = server.available();

  // extract the Rocrail hash
  debug.printf("New Client Rocrail : 0x");
  if (client.connected()) {            // loop while the client's connected
    int16_t rb = 0;                    //!\ Do not change type int16_t
    while (rb != 13)
    {
      if (client.available()) // if there's bytes to read from the client,
        rb = client.readBytes(cBuffer, 13);
    }
    RrHash = ((cBuffer[2] << 8) | cBuffer[3]);

    debug.println(RrHash, HEX);

    // --- register Rocrail on the CAN bus
    CANMessage frame;
    frame.id = (cBuffer[0] << 24) | (cBuffer[1] << 16) | RrHash;
    frame.ext = true;
    frame.len = cBuffer[4];
    for (byte i = 0; i < frame.len; i++)
      frame.data[i] = cBuffer[i + 5];
    const uint32_t ok = ACAN_ESP32::can.tryToSend(frame);

    debugFrame(&frame);
  }
}



//----------------------------------------------------------------------------------------
//   LOOP
//----------------------------------------------------------------------------------------

void loop()
{
  CANMessage frameIn;
  if (ACAN_ESP32::can.receive(frameIn))
  {
    // clear sBuffer
    for (byte el : sBuffer)
      el = 0;

    debug.printf("CAN -> TCP\n\n");
    debugFrame(&frameIn);

    sBuffer[0] = (frameIn.id & 0xFF000000) >> 24;
    sBuffer[1] = (frameIn.id & 0xFF0000) >> 16;
    sBuffer[2] = (frameIn.id & 0xFF00) >> 8; // hash
    sBuffer[3] = (frameIn.id & 0x00FF);      // hash
    sBuffer[4] = frameIn.len;
    for (byte i = 0; i < frameIn.len; i++)
      sBuffer[i + 5] = frameIn.data[i];

    client.write(sBuffer, 13);
  }

  // clear cBuffer
  for (byte el : cBuffer)
    el = 0;


  if (client.available())
  {
    debug.printf("TCP -> CAN\n\n");

    if (client.readBytes(cBuffer, 13) == 13)
    {
      CANMessage frameOut;
      frameOut.id = (cBuffer[0] << 24) | (cBuffer[1] << 16) | RrHash;
      frameOut.ext = true;
      frameOut.len = cBuffer[4];
      for (byte i = 0; i < frameOut.len; i++)
        frameOut.data[i] = cBuffer[i + 5];

      debugFrame(&frameOut);
      const bool ok = ACAN_ESP32::can.tryToSend(frameOut);
    }
  }

  if (!client.connected())
  {
    client.stop();
    Serial.println("Client Disconnected.");
  }
}

void debugFrame(const CANMessage *frame)
{
  debug.print("Hash : 0x");
  debug.println(frame->id & 0xFFFF, HEX);
  debug.print("Response : ");
  debug.println((frame->id & 0x10000) >> 16 ? "true" : "false");
  debug.print("Commande : 0x");
  debug.println((frame->id & 0x1FE0000) >> 17, HEX);
  for (byte i = 0; i < frame->len; i++)
  {
    debug.printf("data[%d] = 0x", i);
    debug.print(frame->data[i], HEX);
    if (i < frame->len - 2)
      debug.print(" - ");
  }
  debug.println("");
  debug.println("-----------------------------------------------------------------------------------------------------------------------------");
  debug.println("");
}


//----------------------------------------------------------------------------------------
