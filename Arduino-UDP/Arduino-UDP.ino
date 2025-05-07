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
  pinMode(LED_BUILTIN, OUTPUT);
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
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  int rxPacketSize = Udp.parsePacket();
  if (rxPacketSize) {
    // Why sizeof(packetBuffer)-1 ?
    // UDP packets are raw byte data, not null-terminated. 
    // The 1024th place will be added null(0) if fully length is used. 
    int rxLen = Udp.read(packetBuffer, sizeof(packetBuffer)-1);
    packetBuffer[rxLen] = 0;

    Serial.print("[");
    Serial.print(Udp.remoteIP());
    Serial.print(":");
    Serial.print(Udp.remotePort());
    Serial.print("] ");
    Serial.println(packetBuffer);
  }

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.toCharArray(packetBuffer, sizeof(packetBuffer));
    Serial.print("[You:");
    Serial.print(localPort);
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