#include <WiFiS3.h>

// WiFi credentials
char wifi_ssid[] = "Perifural.EmbedSys.24";
char wifi_pass[] = "summer-solstice-25";

// IP and Port
unsigned int localPort = 9000;
unsigned int remotePort = 9000;
IPAddress remoteIp(43, 153, 39, 86);

int wifiStatus = WL_IDLE_STATUS;
char packetBuffer[1024];
WiFiUDP Udp;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  
  while (wifiStatus != WL_CONNECTED) {
    Serial.print("Connecting: ");
    Serial.println(wifi_ssid);
    Serial.println();

    wifiStatus = WiFi.begin(wifi_ssid, wifi_pass);
    delay(5000);
  }
  
  Serial.println("Connected");
  printWifiStatus();
  Serial.println();

  Udp.begin(localPort);
  Serial.println("Ready");
  Serial.println();
}

void loop() {
  int rxPacketSize = Udp.parsePacket();
  if (rxPacketSize) {
    int rxLen = Udp.read(packetBuffer, 1023);
    if (rxLen > 0) {
      packetBuffer[rxLen] = 0;
    }

    Serial.print("[");
    Serial.print(Udp.remoteIP());
    Serial.print(":");
    Serial.print(Udp.remotePort());
    Serial.print("] ");
    Serial.println(packetBuffer);

    Udp.beginPacket(remoteIp, remotePort);
    Udp.write(packetBuffer);
    Udp.endPacket();
  }
}

void printWifiStatus() {
  // Print SSID
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // Print IPv4
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // Print signal strength 
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}