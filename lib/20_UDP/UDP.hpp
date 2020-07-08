// UDP variables
WiFiUDP UDP;

unsigned int localPort = 6677;
unsigned int remotePort = 6677;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,

void ProcessPacket(String response)
{
   Serial.println(response);
}
