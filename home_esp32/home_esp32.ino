/* 
Audio and CAT transmission over the internet, 25 UDP packets/s
Home part of "Remote HF mk3" project
Uses a ESP32 WROOM module and a WM8731A codec, use "ESP32 Dev module" in the Arduino IDE
uses a small OLED graphic display, a small speaker and a few switches in parallel 
with a dedicated USB CAT interface.  Includes OTA, over the network softwate updates

Pin usage of ESP32 module:
 debug RD        pin 34 IO3
 debug TD        pin 35 IO1
 CAT TD          pin 29 IO5       should be high @ boot:  ok
 CAT RD          pin 26 IO4
 CAT PTT         pin 37 IO23
 freqa           pin 14 IO12      should be low/floating @ boot: ok
 I2C SCL         pin 13 IO14
 nc              pin 24 IO2       should be low/floating @ boot: ok
 I2C SDA         pin 23 IO15      should be high @ boot: ok
 I2S DIN         pin 33 IO21
 I2S DOUT        pin 36 IO22
 I2S FCLK        pin 11 IO26
 I2S BCLK        pin 10 IO25
 I2S MCLk        pin 25 IO0       should be low @ boot:  ok 
 grn led         pin 31 IO19
 red led         pin 30 IO18
 SWR-fwd         pin 7  IO35      ADC1-ch7, volume ctl
 connect         pin 12 IO26      switch
 freq0           pin 24 IO2       rotary encoder for frequency tuning
 freq1           pin 16 IO13
   
 Written by jean.taeymans@telenet.be
 01/03/20:  on the final hardware testing the various interfaces
 07/03/20:  running with remote, 
                -sound working in both directions
                -CAT working @2400 baud with µBIT-X emulator & with FLDIGI
                -CTL interface working, writing 8 system parameters on OLED
 11/03/20:  improved power indication, volume control on the LS output
              testing with a scheme to accerlerate the CAT, does not work well
              testing the PTT via the DTR f the USN, works ok, the serial port can still be used 
 15/03/20:  transparant CAT link to/ remote does not work with WSJT-X (long delays ?) 
              replaced by a simple terminal driven frequency & mode setting
              using a rotary encoder for fine frequency adjustment, to be improved    
 22/03/20:  2nd trial for a semi transparant CAT, works OK with FLdigi, does not work well with WSJT-X under Ubuntu   
 28/03/20:  final version, bug fixing & connection switch 
 31/03/20:  using small packets at 25 per s
 08/04/20:  some minor aesthetic fixing 
 20/04/20:  adding VOX capability, rationalise the several PTT methods  
 22/04/20:  adding the kill switch (turning the volume all the way down) 
 24/04/20:  correcting I2S bug, no rattle on the audio output anymore  
 20/05/20:  adding the AP fall back mode, no DNS lookup in case of direct WiFi connection to the remote 
 30/05/20:  not using EasyDDNS library, using traditional way, DDNS updates work ok now.  
*/

#include <Arduino.h>
#include "driver/i2s.h"
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "freertos/queue.h"
#include <WebServer.h>
#include <Update.h>
#include <HTTPClient.h>           // test
#include <WiFiMulti.h>            // multiple AP management
#include <Adafruit_GFX.h>         // drivers for the OLED display
#include "SSD1306.h"

// Pin definitions:
#define RD_u 5          // 2nd UART, USB CAT connection
#define TD_u 4    
#define PTT_u 23        // PTT input, DTR of USB CAT connection 
#define I2C_SDA 15      // I2C for WM8731 & OLED
#define I2C_SCL 14
#define I2S_DOUT 21     // I2S for WM8731
#define I2S_DIN 22
#define I2S_FCLK 26
#define I2S_BCLK 25
#define I2S_MCLK 0
#define vol_ctl 35      // analog input, LS volume control
#define con_sw 27       // connect switch
#define freq0 2         // rotary encoder for setting the frequency
#define freq1 13
#define freqa 12
#define G_LED 19        // debug leds
#define R_LED 18

#define oled 0x3c     // I2C address of OLED display
#define codec 0x1a    // I2C address of WM8731 codec

// other defines
#define SAMPLERATE 8000l

//i2s configuration 
int i2s_num = 0; // i2s port number
i2s_config_t i2s_config = {
     .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX),  
     .sample_rate = SAMPLERATE,
     .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
     .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
     .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_LSB),
     .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // high interrupt priority, interrupt level 1
     .dma_buf_count = 8,
     .dma_buf_len = 256, 
     .use_apll = true
  };
i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK, 
    .ws_io_num = I2S_FCLK, 
    .data_out_num = I2S_DOUT, 
    .data_in_num = I2S_DIN  
  };

// Network stuff
const char home_name[] = "home.duckdns.org";      // example, to be adapted according the network
const char remote_name[] = "remote.duckdns.org";  // example, to be adapted according the network
const int udp_h_port = 1234; // port for data transfer to the home device
const int udp_r_port = 1235; // port for data transfer to the remote device
const int tcp_u_port = 1236; // port for OTA updates

// Set gateway and static IP addresses  
IPAddress IP_home(192, 168, 0, 207);       // example, to be adapted according the network
IPAddress IP_remote(192, 168, 0, 208);     // example, to be adapted according the network
IPAddress gateway(192, 168, 0, 1);         // example, to be adapted according the network
IPAddress subnet(255, 255, 255, 0);        // example, to be adapted according the network
IPAddress primaryDNS(123, 123, 123, 123);  // example, to be adapted according the network
IPAddress secondaryDNS(123, 123, 123, 123);// example, to be adapted according the network

// DynDNS client
HTTPClient duckreq;

// OLED 
SSD1306  display(oled, I2C_SDA, I2C_SCL);

// Data structure for access point manager
WiFiMulti wifiMulti;

// define the UDP server, at first for the NTP
WiFiUDP udp;

// OTA update server, need a pin hole for port 1236/tcp through the router
WebServer server(tcp_u_port);       
// Login page 
const char* loginIndex = 
 "<form name='loginForm'>"
    "<table width='20%' bgcolor='A09F9F' align='center'>"
        "<tr>"
            "<td colspan=2>"
                "<center><font size=4><b>ESP32 Login Page</b></font></center>"
                "<br>"
            "</td>"
            "<br>"
            "<br>"
        "</tr>"
        "<td>Username:</td>"
        "<td><input type='text' size=25 name='userid'><br></td>"
        "</tr>"
        "<br>"
        "<br>"
        "<tr>"
            "<td>Password:</td>"
            "<td><input type='Password' size=25 name='pwd'><br></td>"
            "<br>"
            "<br>"
        "</tr>"
        "<tr>"
            "<td><input type='submit' onclick='check(this.form)' value='Login'></td>"
        "</tr>"
    "</table>"
"</form>"
"<script>"
    "function check(form)"
    "{"
    "if(form.userid.value=='on4jrt' && form.pwd.value=='123456789')"
    "{"
    "window.open('/serverIndex')"
    "}"
    "else"
    "{"
    " alert('Error Password or Username') /*displays error message*/"
    "}"
    "}"
"</script>";

// Server Index Page 
const char* serverIndex = 
"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
   "<input type='file' name='update'>"
        "<input type='submit' value='Update'>"
    "</form>"
 "<div id='prg'>progress: 0%</div>"
 "<script>"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  " $.ajax({"
  "url: '/update',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "processData:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "console.log('success!')" 
 "},"
 "error: function (a, b, c) {"
 "}"
 "});"
 "});"
 "</script>";

// global variables
#define snd_buf_size 8000l    // output buffer 1s mono
#define in_bufs 2
#define in_buf_size 320       // 25 small packets per s
#define udp_buf_size 660      // = (snd_buf_size * 2) + 20
#define udp_pac_cnt 640
#define udp_ctl_cnt 641
#define udp_cat_mes 642       // start of cat message, max 5 bytes
#define udp_ctl_mes 647       // start of ctl message, max 13 bytes

#define voxThreshold 10000    // amplitude threshold (absolute value) of VOX trigger, about 0.35Veff

int16_t snd_buf[snd_buf_size];
uint16_t in_buf[in_bufs][in_buf_size];   // input data buffer, buffering two times 0.08s of sound, i.e. twice 640 samples/1280 bytes
uint16_t current_in_buf = 0;             // which data buffer is being used for input and which is being used for output
uint16_t current_out_buf = 0;
uint16_t in_buf_pos = 0;                 // position in the input data buffer, will advance 2 each sample
int16_t silence[2] = { 0, 0};
int16_t temp[2] = { 0, 0};
int16_t synth[2] = { 0, 0};
int16_t snd_buf_w = 0, snd_buf_r = 0;
int16_t in_buf_w = 0, in_buf_r = 0;
int32_t snd_buf_fill;
int32_t in_buf_fill;
boolean snd_on = false, wifi_on = false;
char tmpstr[32], tmpstrs[16];
uint8_t udp_r_buf[udp_buf_size];
uint8_t udp_s_buf[udp_buf_size];
uint16_t rx_pac_cnt = 0, tx_pac_cnt = 0, pac_num = 0, exp_pac_num = 0;
char *pos;
float bat_volt, fpow_fwd, fpow_rfl, rfl_coef, swr;
int16_t temp_rem, pow_fwd, pow_rfl, rem_pac, rssi_rem;
uint8_t vol = 121, old_vol = 121;
uint16_t pot_val = 60558;
boolean ptt_cat = false, ptt_rf = false, ptt_hf = false, ptt_hf_old = false, ptt_tim = false;
boolean ptt_cat_old = false, ptt_dtr_old = false, ptt_vox_old = false, con_sw_old = false;
boolean kill_fl = false;
int16_t vox = 0;
long ptt_time = 0;
#define max_ptt 180000l         // max transmission time, 3min

uint8_t cat_in[5], cat_out[5];
uint8_t cat_mode = 0x01, old_cat_mode = 0x01;
long freq = 14074000l, old_freq = 14074000l, displ_freq;
boolean cat_in_fl = false, cat_out_fl = false, extra_ctl = false;
int16_t puls = 0;

boolean splitActive = false;
boolean vfoAActive = true;
boolean cat_enabled = true;   
byte nullPad[5] = {0,0,0,0,0};
uint16_t cat_cnt, ctl_cnt;

uint16_t accu = 0, adder = 0;

const uint8_t wm8731_regs[12][2] = {  // WM8731 initialization
  { 0x1e, 0x00 },   //  reset
  { 0x0c, 0x10 },   //  power control: all on except output
  { 0x0e, 0x01 },   //  data format: slave mode, 16bit, MSB-First, left justified
  { 0x00, 0x17 },   //  lin vol: 0dB
  { 0x02, 0x17 },   //  rin vol: 0dB
  { 0x04, 0x79 },   //  lhp vol: 0dB
  { 0x06, 0x79 },   //  rhp vol: 0dB
  { 0x08, 0x10 },   //  ana conf: DAC select & line in select
  { 0x0a, 0x00 },   //  dig conf: normal
  { 0x10, 0x00 },   //  sample rate: normal, 48ks/s @ 12288kHz MCLK yields 8ks/s @ 2048kHz
  { 0x12, 0x01 },   //  enable: interface active
  { 0x0c, 0x00 } }; //  power control, all on incl. output

const int16_t sine[256] = { 0, 245, 491, 736, 980, 1224, 1467, 1710, 1951, 2191, 2430, 2667, 2903, 3137, 3369, 3599, 3827, 4052, 4276,
                          4496, 4714, 4929, 5141, 5350, 5556, 5758, 5957, 6152, 6344, 6532, 6716, 6895, 7071, 7242, 7410, 7572, 7730,
                          7883, 8032, 8176, 8315, 8449, 8577, 8701, 8819, 8932, 9040, 9142, 9239, 9330, 9415, 9495, 9569, 9638, 9700,
                          9757, 9808, 9853, 9892, 9925, 9952, 9973, 9988, 9997, 10000, 9997, 9988, 9973, 9952, 9925, 9892, 9853, 9808,
                          9757, 9700, 9638, 9569, 9495, 9415, 9330, 9239, 9142, 9040, 8932, 8819, 8701, 8577, 8449, 8315, 8176, 8032,
                          7883, 7730, 7572, 7410, 7242, 7071, 6895, 6716, 6532, 6344, 6152, 5957, 5758, 5556, 5350, 5141, 4929, 4714,
                          4496, 4276, 4052, 3827, 3599, 3369, 3137, 2903, 2667, 2430, 2191, 1951, 1710, 1467, 1224, 980, 736, 491, 245,
                          0, -245, -491, -736, -980, -1224, -1467, -1710, -1951, -2191, -2430, -2667, -2903, -3137, -3369, -3599, -3827, -4052, -4276,
                          -4496, -4714, -4929, -5141, -5350, -5556, -5758, -5957, -6152, -6344, -6532, -6716, -6895, -7071, -7242, -7410, -7572, -7730,
                          -7883, -8032, -8176, -8315, -8449, -8577, -8701, -8819, -8932, -9040, -9142, -9239, -9330, -9415, -9495, -9569, -9638, -9700,
                          -9757, -9808, -9853, -9892, -9925, -9952, -9973, -9988, -9997, -10000, -9997, -9988, -9973, -9952, -9925, -9892, -9853, -9808,
                          -9757, -9700, -9638, -9569, -9495, -9415, -9330, -9239, -9142, -9040, -8932, -8819, -8701, -8577, -8449, -8315, -8176, -8032,
                          -7883, -7730, -7572, -7410, -7242, -7071, -6895, -6716, -6532, -6344, -6152, -5957, -5758, -5556, -5350, -5141, -4929, -4714,
                          -4496, -4276, -4052, -3827, -3599, -3369, -3137, -2903, -2667, -2430, -2191, -1951, -1710, -1467, -1224, -980, -736, -491, -245 };

const uint8_t message[] = {  // "THE QUICK BROWN FOX JUMPS OVER THE LAZY DOG 0123456789 !"
                          0x08, 0x02, 0x1f, 0x1f, 0x10, 0x14, 0x01, 0x04, 0x17, 0x07, 0x06, 0x0e, 0x0f, 0x04, 0x19, 0x0a, 
                          0x18, 0x13, 0x0c, 0x04, 0x0d, 0x18, 0x1d, 0x04, 0x0b, 0x07, 0x1c, 0x16, 0x05, 0x04, 0x18, 0x1e, 
                          0x01, 0x0a, 0x04, 0x10, 0x14, 0x01, 0x04, 0x12, 0x03, 0x11, 0x15, 0x04, 0x09, 0x18, 0x1a, 0x04,
                          0x04, 0x04, 0x1b, 0x1b, 0x16, 0x17, 0x13, 0x01, 0x0a, 0x10, 0x15, 0x07, 0x06, 0x18, 0x04, 0x1b, 0x0d };


// *********************************************************************************************
// *  ISR function, used for the rotary encoder                                                                         *
// *********************************************************************************************
// for some reason this has to be located in RAM, beware, calls to non RAM located functions are not allowed
void ICACHE_RAM_ATTR rotary(void)
{  // we come here only at falling edge of freq0
  if(digitalRead(freq1)) puls++;
  else puls--;
}

// *********************************************************************************************
// Setting up the environment
// *********************************************************************************************
void setup() {
  uint16_t i, j, err = 0;
  String mess;
  pinMode (R_LED, OUTPUT );
  digitalWrite(R_LED, HIGH);
  pinMode (G_LED, OUTPUT );
  digitalWrite(G_LED, HIGH);
  pinMode (PTT_u, INPUT );
  pinMode (con_sw, INPUT_PULLUP);
  pinMode (freq0, INPUT_PULLUP);
  pinMode (freq1, INPUT_PULLUP);
  pinMode (freqa, INPUT_PULLUP);
  
  analogSetWidth(12);
  analogSetAttenuation(ADC_11db);   
  // starting the serial ports
  //Wire.begin(I2C_SDA, I2C_SCL, 100000);         // creating a I2C port, already done in the SSD1306 library
  Serial.begin(115200);                           // uart connection used for debugging
  Serial1.begin(9600, SERIAL_8N1, RD_u, TD_u);    // uart CAT connection,
  delay(10); 
  Serial.println("\nhome, getting awake ...");  

  display.init();
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);      // splash screen !
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, "20m HOME");
  display.drawString(0, 16, "ON4JRT 22/04/20");  
  display.display();
  Serial.println("Display configured");
  
  wifiMulti.addAP("SSID_AP_1", "password_AP_1");          // example, to be adapted according the network
  wifiMulti.addAP("SSID_AP_2", "password_AP_2");          // example, to be adapted according the network
  wifiMulti.addAP("SSID_AP_3", "password_AP_3");          // example, to be adapted according the network
  wifiMulti.addAP("SSID_AP_4", "password_AP_4");          // example, to be adapted according the network
  wifiMulti.addAP("RemoteHF", "123456789");               //  remoteHF as AP, fall back mode when no friendly AP is around

  //Configures static IP address
  if (!WiFi.config(IP_home, gateway, subnet, primaryDNS, secondaryDNS))  Serial.println("WiFi failed to configure");
  delay(10);
  if(wifiMulti.run() == WL_CONNECTED) {
  wifi_on = true;

  Serial.print("Connected to WiFi AP: "); Serial.print(WiFi.SSID());
  //Serial.print("IP address: ");  Serial.print(WiFi.localIP());
  long rssi = WiFi.RSSI(); // print the received signal strength
  Serial.print("  RSSI: "); Serial.print(rssi); Serial.println(" dBm");
  delay(10);
  Serial.println();

    // DuckDNS.org allows to update with "http://duckdns.org/update/exampledomain/yourtoken" 
    // example, to be adapted according the network
    duckreq.begin( "http://www.duckdns.org/update/remote/nnnnnnnn-nnnn-nnnn-nnnn-nnnnnnnnnnnn?");
    int retcode = duckreq.GET();
    if ( retcode == HTTP_CODE_OK) {
      Serial.print( "DynDNS updated: ");
      String payload = duckreq.getString();
      Serial.println(payload);
      }
    else {
    Serial.println ( "DynDNS update failed");
    }
    duckreq.end(); 

// stuff for the OTA
// return index page which is stored in serverIndex 
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", loginIndex);
  });
  server.on("/serverIndex", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex);
  });
  //handling uploading firmware file 
    server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      // flashing firmware to ESP
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });
  Serial.println("web server for OTA updates ready.");  
  server.begin(); 

  if (WiFi.SSID() != "RemoteHF") {   // do not perform a DNS look up when in directly connected to remote
    if(WiFi.hostByName(remote_name, IP_remote) != 1) Serial.println("DNS look up failed");
    WiFi.hostByName(home_name, IP_home);
  } else Serial.println("direct conection, no DNS lookup");

  display.setFont(ArialMT_Plain_16);
  mess  = String(IP_remote[0]); mess +=".";
  mess += String(IP_remote[1]); mess +=".";
  mess += String(IP_remote[2]); mess +=".";
  mess += String(IP_remote[3]);
  display.drawString(0, 32, mess);
  mess  = String(IP_home[0]); mess +=".";
  mess += String(IP_home[1]); mess +=".";
  mess += String(IP_home[2]); mess +=".";
  mess += String(IP_home[3]);
  display.drawString(0, 48, mess);  
  display.display();

  display.setColor(BLACK);
  display.fillRect(0, 16, 128, 16);          // line 2
  display.setColor(WHITE);   
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 16, WiFi.SSID());
  display.display();
   
 } else {
   wifi_on = false;
   WiFi.mode(WIFI_OFF);
   Serial.println("connection failed");
   display.setFont(ArialMT_Plain_16);
   display.drawString(0, 48, "no WiFi !");  
   display.display();
 }
 delay(2000); // show the splash screen for a while
 
  // pre fill the sound buffer with 1s of a chirping sine wave of 300...1200Hz    
  accu = 0;
  adder = 2400;
  for ( i = 0; i < snd_buf_size; i++) {
    accu += adder;   
    j = accu >> 8;
    snd_buf[i] = sine[j];
    adder++;    
  }
  snd_buf_w = snd_buf_size - 16;
  snd_buf_r = 0;
  Serial.println("sound buffer prefilled"); 

 // initialize WM8731 codec, the official I2C way
  for (i = 0; i < 12; i++) {
    Wire.beginTransmission(codec);
    Wire.write(wm8731_regs[i][0]);
    Wire.write(wm8731_regs[i][1]);
    err += Wire.endTransmission();
    delay(1);
   }
  if(err == 0) Serial.println("codec initialized");
  else Serial.println("codec initialization failed");
  
  // start I2S hardware & driver  
  i2s_driver_install((i2s_port_t)i2s_num, &i2s_config, 0, NULL);  // no events used 
  i2s_set_pin((i2s_port_t)i2s_num, &pin_config);
  // this is black magic to output a 2048 kHz (8kHz*256) clock aka the MCLK on GPIO0 
  REG_WRITE(PIN_CTRL, 0b111111110000);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
  Serial.println("I2S configured @ 8kHz");
  delay(10);  
  
  // run audio task in dedicated task on cpu core 1
  xTaskCreatePinnedToCore(audioTask, "audioTask", 10000, NULL, 10, NULL, 1);
  
  // start the CAT interface
  
  attachInterrupt(digitalPinToInterrupt(freq0), rotary, FALLING);  // start rotary encoder interrupt
    
  // ending the setup
  if (wifi_on) {
  Serial.print("ready! use IP address: ");  Serial.println(WiFi.localIP());  
    udp.begin(udp_h_port);  // start listing to incoming udp packets
  } else {
    Serial.print("ready! no WiFi "); 
  }
  delay(10);
  digitalWrite(R_LED, HIGH);
  digitalWrite(G_LED, HIGH);

}  // end setup

// ********************************************************************************************
// Dedicated audio task, this in in effect a 2nd "loop" 
// ********************************************************************************************
void audioTask(void *) {
  size_t byte_num;
  Serial.println("dedicated audio task created");
  
  while (true) {      // audio task will never end, except with reset or power down

    // check if I2S input DMA buffer is not empty
    byte_num = 4;
    while (byte_num != 0) {
      i2s_read((i2s_port_t)i2s_num, (uint8_t *)&temp[0], 4, &byte_num, 1);  // read sound sample left & right
      in_buf[current_in_buf][in_buf_pos] = temp[0];                         // only left sample is stored
      if (byte_num != 0) {                                                  // check if indeed read, if so try again
        in_buf_pos++;
        if (in_buf_pos == in_buf_size) {
          in_buf_pos = 0;
          if (current_in_buf == 0) current_in_buf = 1;
          else current_in_buf = 0;
        }
      }
    }

  // check if there is enough data to start the audio output stream
    snd_buf_fill = (int32_t)snd_buf_w - (int32_t)snd_buf_r;
    if (snd_buf_fill < 0 ) snd_buf_fill += snd_buf_size;
    if (snd_buf_fill > (snd_buf_size / 2)) { snd_on = true; }
    if (snd_buf_fill < (snd_buf_size / 16)) { snd_on = false; }
    //Serial.println(snd_buf_fill);   // debugging through the serial plotter, very CPU intensive !
  
  // check if I2S output DMA buffer needs a refill
  if (wifi_on) {
      if (snd_on) {
        byte_num = 4;
        while (byte_num != 0) {
          temp[0] = snd_buf[snd_buf_r];
          temp[1] = temp[0];
          i2s_write((i2s_port_t)i2s_num, (uint8_t *)&temp[0], 4, &byte_num, 1);   // write sound sample left & right
          if (byte_num != 0) {                                                    // check if indeed written, if so try once again
            snd_buf_r++;
            if (snd_buf_r == snd_buf_size) snd_buf_r = 0;
          }
        }
      } else {
        byte_num = 4;
        while (byte_num != 0) {
          i2s_write((i2s_port_t)i2s_num, (uint8_t *)&silence[0], 4, &byte_num, 1);  // write silence left & right
        }
      }
    } else {  // if no WiFi we will output a synthezized signal as a debug feature
      byte_num = 4;
      while (byte_num != 0) {
        i2s_write((i2s_port_t)i2s_num, (uint8_t *)&synth[0], 4, &byte_num, 1);  // write sample left & right
        if (byte_num != 0) {  // generate the next sample of a continuous single tone of 785Hz
          accu += adder;
          synth[0] = sine[(accu >> 8)];
          synth[1] = synth[0];
        }
      }
    }
    delay(1);
 }
  vTaskDelete(NULL);
}

// ********************************************************************************************
// Recurrent part of the program, ends with reset or power down
// ********************************************************************************************
void loop() {
  uint16_t i, j;
// structure of a packet, 660 bytes in total:
  // bytes 0...639 :    320 16bit sound samples, 8000 samples/s
  // byte 640 :         packet count [0...255]
  // byte 641 :         bits 7...4: number of CTL bytes [0...13], bit 3: echo of PTT state, bits 0...2: number of CAT bytes [0...5], 
  // byte 642...646 :   CAT bytes, max 5
  // byte 647...659 :   CTL bytes, max 13
  
//  ************************* handling an incoming UDP packet  ********************************
 // if there's data available, read a packet
  int packetSize = udp.parsePacket();
  if (packetSize == udp_buf_size) {
    udp.read(udp_r_buf, udp_buf_size);
    rx_pac_cnt++;     // increment the received packet count
    
    // handle sound part
    for (i = 0; i < in_buf_size; i++) {  
      // transform the first 1280/640 bytes into 640/320 sound samples 
      snd_buf[snd_buf_w] = (uint16_t)udp_r_buf[i * 2] << 8;
      snd_buf[snd_buf_w] += (uint16_t)udp_r_buf[(i * 2) + 1];
      snd_buf_w++;       
      if (snd_buf_w == snd_buf_size) snd_buf_w = 0; 
    }   

    // decode the control bytes @ positions 1280 & 1281 or 640 & 641
    pac_num = (uint16_t)udp_r_buf[udp_pac_cnt];
    if (exp_pac_num != pac_num) Serial.println("!");      // debugging, print "!" when not expected packet #
    exp_pac_num = pac_num + 1;
    if (exp_pac_num > 255) exp_pac_num = 0;
    ctl_cnt = (uint16_t)(udp_r_buf[udp_ctl_cnt] >> 4);
    if ((udp_r_buf[udp_ctl_cnt] & 0x08) == 0x08) ptt_rf = true;
    else ptt_rf = false;
    cat_cnt = (uint16_t)(udp_r_buf[udp_ctl_cnt] & 0x07);
    
    digitalWrite(G_LED, !digitalRead(G_LED));

    // handle the CAT part     
    if (cat_cnt == 5) {                       // display frequency & status response
      displ_freq = readFreq(&udp_r_buf[udp_cat_mes]);
      uint16_t displ_M = displ_freq / 1000000;
      uint16_t displ_k = (displ_freq % 1000000) / 1000;
      uint16_t displ_u = displ_freq % 1000;
      if (ptt_rf == true) {
        if (udp_r_buf[udp_cat_mes + 4] == 0x01) sprintf(tmpstrs, "%2d.%03d.%03dHz U", displ_M, displ_k, displ_u);
        else sprintf(tmpstrs, "%2d.%03d.%03dHz L", displ_M, displ_k, displ_u);
      } else {
        if (udp_r_buf[udp_cat_mes + 4] == 0x01) sprintf(tmpstrs, "%2d.%03d.%03dHz u", displ_M, displ_k, displ_u);
        else sprintf(tmpstrs, "%2d.%03d.%03dHz l", displ_M, displ_k, displ_u);
      }      
      display.setColor(BLACK);
      display.fillRect(0, 0, 128, 16);          // line 1
      display.setColor(WHITE);   
      display.setFont(ArialMT_Plain_16);
      display.drawString(0, 0, tmpstrs);
      display.display();    
    }
    /*
    // play with leds during receiving
    if ((digitalRead(PTT_u)) && (ptt_fl == false)) {        // receiving
      digitalWrite(G_LED, !digitalRead(G_LED));
      digitalWrite (R_LED, HIGH);
    } else {                                               // transmitting
      digitalWrite (R_LED, !digitalRead(R_LED));
      digitalWrite (G_LED, HIGH);
    }  
    */
    // handle the ctl part
    if (ctl_cnt != 0) {
      for (j = 0; j < ctl_cnt; j++) tmpstr[j] = udp_r_buf[udp_ctl_mes + j];
      tmpstr[ctl_cnt] = 0;
      extra_ctl = false;
             
      pos = strstr(tmpstr, "B=");                  // battery voltage and temperature
      if ( pos != NULL) {
        pos += 2;
        bat_volt = (float)atoi(pos) / 1000.;
        pos = strstr(tmpstr, "T=");	
        if ( pos != NULL) {       // pos == NULL is still possible, if message was broken
          pos += 2;
          temp_rem = atoi(pos);
          sprintf(tmpstrs, "%6.3fV    %d'C", bat_volt, temp_rem);
          display.setColor(BLACK);
          display.fillRect(0, 48, 128, 16);           // line 4 (bottom)
          display.setColor(WHITE);   
          display.setFont(ArialMT_Plain_16);
          display.drawString(0, 48, tmpstrs);
          display.display(); 
          extra_ctl = true;
        }
      }  

      pos = strstr(tmpstr, "Y=");                   // WiFi RSSI value
      if ( pos != NULL) { 
        extra_ctl = true; 
        if (ptt_rf == false) {                      // only while receiving  
          pos += 2;  
          rssi_rem = atoi(pos);
          sprintf(tmpstrs, "%ddBm   %ddBm", rssi_rem, (int)WiFi.RSSI());
          display.setColor(BLACK);
          display.fillRect(0, 32, 128, 16);         // line 3
          display.setColor(WHITE);   
          display.setFont(ArialMT_Plain_16);
          display.drawString(0, 32, tmpstrs);
          display.display();
        }     
      }
                                               
      pos = strstr(tmpstr, "P=");                 // forward and reflected power
      if ( pos != NULL) {  
        extra_ctl = true;
        if (ptt_rf == true) {                     // only while transmitting  
          pos += 2;  
          pow_fwd = atoi(pos);
          pos = strstr(tmpstr, "/");
          pos += 1;
          pow_rfl = atoi(pos);
        // Using ADC of remote: 12bit, 4095 adcnts = 3.2Vp (detected) or 1.13Vrms HF
        // using 1/22 current ratio this equates to 0.5Arms in 50Ohm or 12.5W 
          fpow_fwd = ((float)pow_fwd) / 82.73;   // calibrated
          fpow_rfl = ((float)pow_rfl) / 82.73;   // calibrated
          fpow_fwd = fpow_fwd * fpow_fwd / 50.0;
          fpow_rfl = fpow_rfl * fpow_rfl / 50.0;
          if (fpow_fwd > 1.0) {
            rfl_coef = fpow_rfl / fpow_fwd;
            if (rfl_coef > 0.81 ) rfl_coef = 0.81;  // limit SWR indication to ca. 10.0
            swr = (1 + rfl_coef) / (1 - rfl_coef);
          } else {
            swr = 1.0;   
          }
          sprintf(tmpstrs, "%4.1fW    %3.1fSWR", fpow_fwd, swr);
          display.setColor(BLACK);
          display.fillRect(0, 32, 128, 16);        // line 3 
          display.setColor(WHITE);   
          display.setFont(ArialMT_Plain_16);
          display.drawString(0, 32, tmpstrs);
          display.display();
        }
      }
      // display received packet count: home -> remote
      pos = strstr(tmpstr, "Z=");        
      if ( pos != NULL) {   
        pos += 2;  
        rem_pac = atoi(pos);
        sprintf(tmpstrs, "%d/K", rem_pac);
        display.setColor(BLACK);
        display.fillRect(0, 16, 64, 16);          // line 2, left part
        display.setColor(WHITE);   
        display.setFont(ArialMT_Plain_16);
        display.drawString(0, 16, tmpstrs);
        display.display();
        extra_ctl = true;
      }
      // detect the response of a kill request
      pos = strstr(tmpstr, "Bye");        
      if ( pos != NULL) {   
        sprintf(tmpstrs, "   Bye Bye");
        display.setColor(BLACK);
        display.fillRect(0, 0, 128, 16);          // line 1
        display.setColor(WHITE);   
        display.setFont(ArialMT_Plain_16);
        display.drawString(0, 0, tmpstrs);
        display.display();
        extra_ctl = true;
      }
      // if nothing has been parsed in a non-empty ctl message, just print it
      if (!extra_ctl) Serial.println(tmpstr);
    } 
  }
  delay(1);
  
//  ************************* handling an outgoing UDP packet  ********************************
  // check if we're ready to send a buffer of samples over wifi
  if (current_out_buf != current_in_buf) {

    // handling the sound
    for (i = 0; i < in_buf_size; i++) {
      udp_s_buf[i * 2] = (uint8_t)(in_buf[current_out_buf][i] >> 8);
      udp_s_buf[(i * 2) + 1] = (uint8_t)in_buf[current_out_buf][i]; 
      // take a running average 1/256 of the signal:  works fine, 
      // during Rx the running average is ca. 250, during Tx (max power) it is ca. 15000.
      vox = vox + abs((int16_t)(in_buf[current_out_buf][i]) >> 8) - (vox >> 8); 
    }
   
    if  ((tx_pac_cnt % 16) == 1) {           // ca. 1.5 times per sec !
        catFreqUpdate();
    } else {
    //  update the frequency if needed
      if (freq != old_freq) {
        writeFreq(freq, cat_out);
        cat_out[4] = 0x01;
        cat_out_fl = true;
        old_freq = freq;
      }
    // update the mode if needed
      if (cat_mode != old_cat_mode) {
        cat_out[0] = cat_mode;
        cat_out[1] = 0x00;
        cat_out[2] = 0x00; 
        cat_out[3] = 0x00;       
        cat_out[4] = 0x07;
        cat_out_fl = true;
        old_cat_mode = cat_mode;
      }
    }    
    
    udp_s_buf[udp_pac_cnt] = (uint8_t)tx_pac_cnt;  // use position 1280/640 to number the packets 0...255
 
    // send the PTT signal (from the several methods) to the remote
    if (ptt_hf == true) udp_s_buf[udp_ctl_cnt] = 0x08; 
    else udp_s_buf[udp_ctl_cnt] = 0x00; 
   
   // send, if needed, a 5 bytes command to the remote
    if (cat_out_fl) {
      cat_out_fl = false;
      for (i = 0; i < 5; i++) {
        udp_s_buf[udp_cat_mes + i] = cat_out[i];
      }
      udp_s_buf[udp_ctl_cnt] += 0x05;
    } 
    
    // append with the data to be sent to the solar controller
    if (i = Serial.available()) { // 13 chars max
      if (i > 13) i = 13;   // limit the length
      Serial.readBytes(&udp_s_buf[udp_ctl_mes], i);  // hopefully at the right place
      udp_s_buf[udp_ctl_cnt] |= (char)(i << 4) & 0xf0; 
    } else {
      if (kill_fl) {
        udp_s_buf[udp_ctl_mes] = 'K';
        udp_s_buf[udp_ctl_mes + 1] = '?';
        udp_s_buf[udp_ctl_mes + 2] = '\r';
        udp_s_buf[udp_ctl_cnt] |= 0x30;
      }
    }
      
   // do the actual sending only if the connection switch is on
   if (digitalRead(con_sw)) {
     // send everything in a single packet
      udp.beginPacket(IP_remote, udp_r_port);
      udp.write(udp_s_buf, udp_buf_size);      
      udp.endPacket(); 
      tx_pac_cnt++;  
      digitalWrite(R_LED, !digitalRead(R_LED));
   }
   
   // check LS volume adjustment
   // volume potmeter is evaluated every 80ms, a running average of 1/16 
   // shoud equate to a LPF of about 1Hz, enough to eliminate the noise
    pot_val = pot_val + analogRead(vol_ctl) - (pot_val >> 4);
    vol = (uint8_t)map(pot_val, 0, 65535, 48, 127);
    if (vol != old_vol) {    
      Wire.beginTransmission(codec);
      Wire.write(0x04);
      Wire.write(vol);
      Wire.endTransmission();
      old_vol = vol;
      if (vol < 50) kill_fl = true;    // kill the remote if vol is turned full down
      else kill_fl = false;
    }

 if (digitalRead(con_sw)) {
    // display received packet count: remote -> home
    if ((tx_pac_cnt % 1024) == 0) {             
      sprintf(tmpstrs, " %d/K", rx_pac_cnt);
      rx_pac_cnt = 0;
      display.setColor(BLACK);
      display.fillRect(64, 16, 64, 16);           // line 2, right part
      display.setColor(WHITE);   
      display.setFont(ArialMT_Plain_16);
      display.drawString(64, 16, tmpstrs);
      display.display();
    }
 }
    current_out_buf = current_in_buf;
  }
  delay(1);

//  ************************* other stuff  ********************************
// handle the PTT status, check the transitions of the serveral PTT methods
  if ((digitalRead(PTT_u) == LOW) && (ptt_dtr_old == false)) {  // DTR asserted
    ptt_hf = true;
    ptt_dtr_old = true;
    Serial.println("DTR up");
  }
  if ((digitalRead(PTT_u) == HIGH) && (ptt_dtr_old == true)) {  // DTR desasserted
    ptt_hf = false;
    ptt_dtr_old = false;
    Serial.println("DTR down");
  }
  if ((vox > voxThreshold) && (ptt_vox_old == false)) {         // vox detected      
    ptt_hf = true;
    ptt_vox_old = true;
    Serial.println("VOX up");
  }
  if ((vox < (voxThreshold / 2)) && (ptt_vox_old == true)) {    // vox dropped    
    ptt_hf = false;
    ptt_vox_old = false;
    Serial.println("VOX down");
  } 
  if ((ptt_cat == true) && (ptt_cat_old == false)) {            // CAT PTT command      
    ptt_hf = true;
    ptt_cat_old = true;
    Serial.println("CAT up cmd");
  }
  if ((ptt_cat == false) && (ptt_cat_old == true)) {            // CAT un-PTT command    
    ptt_hf = false;
    ptt_cat_old = false;
    Serial.println("CAT down cmd");
  } 
  if ((digitalRead(con_sw) == HIGH) && (con_sw_old == false)) {  // starting a new session
    ptt_hf = false; 
    con_sw_old = true;
    Serial.println("con switch up");
  }
  if ((digitalRead(con_sw) == LOW) && (con_sw_old == true)) {    // ending a session
    con_sw_old = false;
    Serial.println("con switch down");
  }
  if ((ptt_tim == true) && ((millis()- ptt_time) > max_ptt)) {   // timer expired
    ptt_hf = false;
    Serial.println("time up");
  }
  if ((ptt_hf == true) && (ptt_hf_old == false)) {               // preset timer
    ptt_time = millis();
    ptt_tim = true;
    Serial.println("PTT on");
  }
  if ((ptt_hf == false) && (ptt_hf_old == true)) {               // for info
    Serial.println("PTT off");
    ptt_tim = false;
  }
  ptt_hf_old = ptt_hf;
  
// handle the rotary encoder for adjusting the frequency
  if ((puls > 3) || (puls < -3)) puls = 0;  // an atempt to eliminate the bouncing 
  if (puls != 0) {
    if (digitalRead(freqa)) freq += puls * 10;    // step 10Hz
    else freq += puls * 1000;                     // step 1000Hz
    puls = 0;
  }
  // handle the CAT engine on Serial1
  CATcheck();
  
  // check the OTA update server and DDNS activity
  if (wifi_on) {
    server.handleClient();
  }
  delay(1);
}  // end loop
