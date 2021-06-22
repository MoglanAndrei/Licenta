#include <Arduino.h>
#include "WiFi.h"
#include "time.h"
#include <arduinoFFT.h>
#include <driver/i2s.h>
#include <AC101.h>
#include <math.h>
#include <TimeLib.h>
#include <WiFiUdp.h>
#include <WiFiClientSecure.h>
#include <arduinoFFT.h>
#include <driver/i2s.h>

#define WIFI_NETWORK "TP-LINK_6EFC00" // se adauga numele retelei WIFI
#define WIFI_PASSWORD "moscraciun1997" // se adauga parola retelei wifi
#define WIFI_TIMEOUT_MS 20000 //timeout de reconectare
#define IIC_CLK                     32
#define IIC_DATA                    33

#define I2S_WS 26
#define I2S_SD 35
#define I2S_SCK 27





void connectToWiFi()
{
  Serial.print("Connect to WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_NETWORK, WIFI_PASSWORD);

  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED && millis()-startAttemptTime<WIFI_TIMEOUT_MS)
  {
    Serial.print(".");
    delay(100);
  }

  if(WiFi.status()!=WL_CONNECTED)
  {
    Serial.print("Failed!");
  }else{
    Serial.print("Connected!");
    Serial.println(WiFi.localIP());
  }
}
arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
const i2s_port_t I2S_PORT = I2S_NUM_0;
static AC101 ac;
static const char ntpServerName[] = "us.pool.ntp.org";
const int timeZone = 0; //UTC
static WiFiUDP Udp;
unsigned long previousMillis = 0;        // will store last time LED was updated

const long interval = 16000;           // interval at which to blink (milliseconds)  

//Connect to DB

const String node_name = "audio.memorandului";
char url[31] = "/audio.memorandului/_doc";
const char host[] = "8f9677360fc34e2eb943d737b2597c7b.us-east-1.aws.found.io";
const uint16_t httpsPort = 9243;
const String userpass = "";
//

const int BLOCK_SIZE = 1024;
const double samplingFrequency = 44100;
double vReal[BLOCK_SIZE];
double vImag[BLOCK_SIZE];
const int i2s_num = 0;
int retStat = 0;
int32_t sampleIn = 0;
String timestamp;
time_t getNtpTime();
uint32_t t_ms;
uint32_t start_mills;
String run_mills;
int milis_chars;


unsigned long getTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return(0);
  }
  time(&now);
  return now;
}

void setup() {
    Serial.begin(115200);

  Serial.println("Configuring I2S...");
  esp_err_t err;
    Serial.printf("Connect to AC101 codec... ");
  while (not ac.begin(IIC_DATA, IIC_CLK))
  {
    Serial.printf("Failed!\n");
    delay(1000);
  }
  Serial.println("Setup I2S ...");
  delay(1000);
  
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);
    //This pulls in a bunch of samples and does nothing, its just used to settle the mics output
  for (retStat = 0; retStat < BLOCK_SIZE * 2; retStat++)
  {
    i2s_pop_sample((i2s_port_t)i2s_num, (char*)&sampleIn, portMAX_DELAY);
    delay(1);
  }
  Serial.print("Connect to WiFi...");
  connectToWiFi();
  setSyncProvider(getNtpTime);
  setSyncInterval(300);
  configTime(0, 0, ntpServerName);
    
  
}
void loop() {
  sampleIn = 0;
  double data[BLOCK_SIZE] = {0};
  uint8_t counter = 0;
  String httpRequest;
  char ts[23];
  unsigned long currentMillis = millis();
  WiFiClientSecure client;
  client.setInsecure();

  sprintf(ts, "\"%04d-%02d-%02dT%02d:%02d:%02dZ\"\0", year(), month(), day(), hour(), minute(), second());
   if (timestamp != ts){
      timestamp = ts;
  }
  Serial.println(timestamp);
  for (uint16_t i = 0; i < BLOCK_SIZE; i++)
  {
    //this reads 32bits as 4 chars into a 32bit INT variable
    i2s_pop_sample((i2s_port_t)i2s_num, (char*)&sampleIn, portMAX_DELAY);
    //this pushes out all the unwanted bits as we only need right channel data.
    sampleIn >>= 14;
    vReal[i] = sampleIn;
    vImag[i] = 0.0; //Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
  }
  FFT.Windowing(vReal, BLOCK_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, BLOCK_SIZE, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, BLOCK_SIZE);
  double x;
  double v;
  FFT.MajorPeak(vReal, BLOCK_SIZE, samplingFrequency, &x, &v);
//  Serial.print(x, 6);
//  Serial.print(", ");
//  Serial.println(v, 6);
//double k[BLOCK_SIZE];
  double *k = (double *)malloc(sizeof(double) * 9000);
  double p = 0;
  double dB = 20*log10(x);
  for(uint16_t i = 0; i < BLOCK_SIZE; i++)
      {
       k[i] = dB; 
      }
       
  for(uint16_t i = 0; i < BLOCK_SIZE; i++){
        p = max(k[i], p);
      }
   Serial.println(p);

   if (!client.connect(host, httpsPort)) {
    delay(2500);
    Serial.println("connection failed");
   }
    else
    {
      Serial.println("connection ");
    } 
     String data_to_upload = "{\"x\":" + String(p) + ","  +
                          "\"timestamp\":" + timestamp + ",\"lat\":45.6350585" + ",\"lon\":25.6168933" + "," + "\"zone\":" + "\"Racadau\"" +"}";
 

   httpRequest = String("POST ") + url + " HTTP/1.1\r\n" +
                      "Host: " + host +":" + httpsPort+"\r\n" + 
                      "User-Agent: esp-idf/1.0 esp32\r\n" +
                      "Authorization: Basic " + "ZWxhc3RpYzpBV2J0bUdkYTJRN0JJMmJZcGRqeUY0cWQ=" + "\r\n" +
                      "Connection: close\r\n" +
                      "Content-Type: application/json\r\n" +
                      "Content-Length: " + data_to_upload.length() +
                      "\r\n\r\n" + data_to_upload;
        Serial.println(httpRequest);
        
        client.print(httpRequest);

  while (client.available()) {
      String line = client.readStringUntil('\n');
      if (line == "\r") {
        break;
      }
    }

    String line = client.readStringUntil('\n');
//    Serial.print("Reply was:");
//    Serial.println(line);
//
//    Serial.println(".");
    
    client.stop();
//    delay(5000);
   free(k);
   k = nullptr;
   
   delay(15000);
   
   
}

void i2s_install(){
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 44100,
    .bits_per_sample = i2s_bits_per_sample_t(32),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = 0, // default interrupt priority
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

void i2s_setpin(){
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };

  i2s_set_pin(I2S_PORT, &pin_config);
}

/*-------- NTP code ----------*/
const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets
time_t getNtpTime()
{
  IPAddress ntpServerIP; // NTP server's ip address
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  Serial.print(ntpServerName);
  Serial.print(": ");
  Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}
// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}
