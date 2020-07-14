/*
  Audio and CAT transmission over the internet, 25 UDP packets/s in both directions
  The controller part of the "Remote HF mk3" project, wich includes a µBIT-X transceiver
  Uses a ESP32 WROOM module and a WM8731A codec, use "ESP32 Dev module" in the Arduino IDE
  Includes OTA, over the network software updates
  - connected to the µBIT-X with an isolated serial port, Rd, TD & PTT, emulating the CAT interface
  - connected to the µBIT-X with an isolated audio interface, receive and transmit
  - connected to the solar controller module for timing and battery supervision
  The Remote HF mk3 will attempt to connect to the strongest of a small number of APs when in range,
  failing that it will be become in itself an AP for local connection to the home device
  
  Pin usage of ESP32 module:
  debug RD        pin 34 IO3
  debug TD        pin 35 IO1
  Solar RD        pin 29 IO5       should be high @ boot: ok
  Solar TD        pin 26 IO4
  µBIT-X RD       pin 9  IO33
  µBIT-X TD       pin 8  IO32
  µBIT-X PTT      pin 37 IO23
  nc              pin 14 IO12      should be low/floating @ boot: ok
  I2C SCL         pin 13 IO14
  nc              pin 24 IO2       should be low/floating @ boot: ok
  I2C SDA         pin 23 IO15      should be high @ boot: ok
  I2S DOUT        pin 33 IO21
  I2S DIN         pin 36 IO22
  I2S FCLK        pin 11 IO26
  I2S BCLK        pin 10 IO25
  I2S MCLk        pin 25 IO0       should be low @ boot:  ok
  BITX-on         pin 12 IO27
  grn led         pin 31 IO19
  red led         pin 30 IO18
  SWR-fwd         pin 7  IO35      ADC1-ch7
  SWR-rev         pin 6  IO34      ADC1-ch6
  debug           pin    IO13      not used

  Written by jean.taeymans@telenet.be
  05/02/20:   working on the development hardware, audio loopback through a 5s buffer
              issues: - does not boot well, need to decouple the hardware
                      - WM8731 does not work with the regular I2C; need the bit banged version
                      - does not work when using IO12/IO14 for the I2C, TBC
  14/02/20:   still on the development hardware, same functionality, including OTA updates
              problems with booting and with I2C solved, WM8731 works with regular Wire library
              must use IO14/IO15 for the I2C
  16/02/20:   still on the development hardware, including interaction with solar controller
              and mail sending client
  28/02/20:   on the final hardware testing the various interfaces
  07/03/20:   running with remote,
              -sound working in both directions
              -CAT working @2400 baud with µBIT-X emulator & with FLDIGI
              -CTL interface working, writing 8 system parameters on OLED
  11/03/20:   improved power indication, volume control on the LS output
              fully transparant CAT does not work very well @ 4800 baud, due to transmission delays
  31/03/20:   final version, including conditional transmission and the use of 25 small packets per s
  08/04/20:   some minor aesthetic fixings  
  22/04/20:   again some minor fixings related to stand alone mode, small delay on the PTT
  24/04/20:   correcting I2S bug, no rattle on the audio out anymore
  20/05/20:   adding the fall back to AP mode, when no real APs are in range
  15/06/20:   not using EasyDDNS library, using traditional way, DDNS updates work ok now.  
*/

#include <Arduino.h>
#include "driver/i2s.h"
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>            // F. Weinberg 3.1, NTP client
#include "freertos/queue.h"
#include <WebServer.h>
#include <Update.h>
#include "ESP32_MailClient.h"     // Mobizt, 2.1.4, mail client for sending
#include <WiFiMulti.h>            // multiple AP management, take the strongest available

// Pin definitions:
#define RD_s 5          // 3rd UART, connection with the Solar/Battery controller
#define TD_s 4
#define RD_u 33         // 2nd UART, CAT connection with µBIT-X
#define TD_u 32
#define PTT_u 23        // PTT output to µBIT-X
#define BITX_u 27       // power switch for µBIT-X
#define I2C_SDA 15      // I2C for WM8731
#define I2C_SCL 14
#define I2S_DOUT 21     // I2S for WM8731
#define I2S_DIN 22
#define I2S_FCLK 26
#define I2S_BCLK 25
#define I2S_MCLK 0
#define SWR_fwd 35      // analog inputs for SWR bridge 
#define SWR_rfl 34
#define G_LED 18        // debug leds
#define R_LED 19
#define DEBUG 13        // debug mode, no used for now

// other defines
#define SAMPLERATE 8000l
#define codec 0x1a    // I2C address of WM8731 codec

//i2s configuration
int i2s_num = 0;           // i2s port number
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
const char home_name[] = "home.duckdns.org";              // to be adapted according to own configuration
const char remote_name[] = "remote.duckdns.org";          // to be adapted according to own configuration
const char AP_ssid[] = "RemoteHF";                        // in case no friendly AP is present
const char AP_password[] = "123456789";
const int udp_h_port = 1234; // port for data transfer to the home device
const int udp_r_port = 1235; // port for data transfer to the remote device
const int tcp_u_port = 1236; // port for OTA updates

// Set gateway and static IP addresses  
IPAddress IP_home(192, 168, 1, 207);        // example, to be adapted according to own configuration
IPAddress IP_remote(192, 168, 1, 208);      // example, to be adapted according to own configuration
IPAddress gateway(192, 168, 1, 1);          // example, to be adapted according to own configuration
IPAddress subnet(255, 255, 255, 0);         // example, to be adapted according to own configuration
IPAddress primaryDNS(123, 123, 123, 123);    // example, to be adapted according to own configuration
IPAddress secondaryDNS(123, 123, 123, 123); // example, to be adapted according to own configuration

// Define NTP Client to get the network time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
// Variables to save date and time
String formattedDate;
String dayStamp;
String timeStamp;

// DynDNS client
HTTPClient duckreq;

// Data structure for access point manager
WiFiMulti wifiMulti;

// define the UDP server
WiFiUDP udp;

// The Email Sending data object contains config and data to send
SMTPData smtpData;

// OTA update server, need a pin hole for port 1236/tcp through the router in order to use this externally
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

#define snd_buf_size 8000l // output buffer 1s mono
#define in_bufs 2          // inupt buffer work in tandem

#define in_buf_size 320    // 25 small packets per s, 16000 bytes or 8000 samples per s
#define udp_buf_size 660   // = (snd_buf_size * 2) + 20, 20 extra bytes for houskeeping
#define udp_pac_cnt 640    // one byte for numbering packets
#define udp_ctl_cnt 641    // controlling the houskeeping bytes
#define udp_cat_mes 642    // start of cat message, max 5 bytes, FT-817 stuyle 
#define udp_ctl_mes 647    // start of ctl message, max 13 bytes, 

int16_t snd_buf[snd_buf_size];
int16_t in_buf[in_bufs][in_buf_size];  // input data buffer, buffering two times 0.08s of sound, i.e. twice 640 samples/1280 bytes
uint16_t current_in_buf = 0;            // which data buffer is being used for input and which is being used for output
uint16_t current_out_buf = 0;
uint16_t in_buf_pos = 0;                // position in the input data buffer, will advance 2 each sample
int16_t silence[2] = { 0, 0};
int16_t temp[2] = { 0, 0};
int16_t synth[2] = { 0, 0};
int16_t snd_buf_w = 0, snd_buf_r = 0;
int16_t in_buf_w = 0, in_buf_r = 0;
int32_t snd_buf_fill;
int32_t in_buf_fill;
boolean snd_on = false, debug_on = false, ptt_rf = false, ptt_rf_old = false;
long ptt_tim = 0;
#define ptt_del 330l   // wait for audio buffer to emtpy before switching the ptt

char tmpstr[16];
uint8_t udp_r_buf[udp_buf_size];
uint8_t udp_s_buf[udp_buf_size];
uint16_t rx_pac_cnt = 0, tx_pac_cnt = 0, pac_num = 0;
uint16_t fwd_pow, rfl_pow;
uint16_t cat_cnt, ctl_cnt;

uint8_t in_sol, in_sol_buf[16];
uint16_t in_sol_pnt = 0;
boolean in_sol_fl = false;

#define no_con_time 10000   // time with no incoming UDP packets before remote stops the sending of UDP packets
long connect_time = 0;
boolean BITX_on = false, old_BITX_on = false;

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
  { 0x10, 0x00 },   //  sample rate: normal, 48ks/s @ 12288kHz MCLK yieds 8ks/s @ 2048kHz
  { 0x12, 0x01 },   //  enable: interface active
  { 0x0c, 0x00 }    //  power control, all on incl. output
}; 

uint16_t accu = 0, adder = 0;

const int16_t sine[256] = {   0,   736,  1472,  2207,  2941,  3672,  4402,  5129,  5853,  6573,  7289,  8001,  8709,  9410, 10107, 10797, 
                          11481, 12157, 12827, 13488, 14142, 14787, 15423, 16050, 16667, 17274, 17871, 18457, 19032, 19595, 20147, 20686, 
                          21213, 21727, 22229, 22716, 23190, 23650, 24096, 24528, 24944, 25346, 25732, 26103, 26458, 26797, 27120, 27426, 
                          27716, 27990, 28246, 28486, 28708, 28913, 29101, 29271, 29424, 29558, 29675, 29774, 29856, 29919, 29964, 29991,
                          30000, 29991, 29964, 29919, 29856, 29774, 29675, 29558, 29424, 29271, 29101, 28913, 28708, 28486, 28246, 27990,
                          27716, 27426, 27120, 26797, 26458, 26103, 25732, 25346, 24944, 24528, 24096, 23650, 23190, 22716, 22229, 21727,
                          21213, 20686, 20147, 19595, 19032, 18457, 17871, 17274, 16667, 16050, 15423, 14787, 14142, 13488, 12827, 12157,
                          11481, 10797, 10107,  9410,  8709,  8001,  7289,  6573,  5853,  5129,  4402,  3672,  2941,  2207,  1472,   736,
                              0,  -736, -1472, -2207, -2941, -3672, -4402, -5129, -5853, -6573, -7289, -8001, -8709, -9410,-10107,-10797,
                         -11481,-12157,-12827,-13488,-14142,-14787,-15423,-16050,-16667,-17274,-17871,-18457,-19032,-19595,-20147,-20686,
                         -21213,-21727,-22229,-22716,-23190,-23650,-24096,-24528,-24944,-25346,-25732,-26103,-26458,-26797,-27120,-27426,
                         -27716,-27990,-28246,-28486,-28708,-28913,-29101,-29271,-29424,-29558,-29675,-29774,-29856,-29919,-29964,-29991,
                         -30000,-29991,-29964,-29919,-29856,-29774,-29675,-29558,-29424,-29271,-29101,-28913,-28708,-28486,-28246,-27990,
                         -27716,-27426,-27120,-26797,-26458,-26103,-25732,-25346,-24944,-24528,-24096,-23650,-23190,-22716,-22229,-21727,
                         -21213,-20686,-20147,-19595,-19032,-18457,-17871,-17274,-16667,-16050,-15423,-14787,-14142,-13488,-12827,-12157,
                         -11481,-10797,-10107, -9410, -8709, -8001, -7289, -6573, -5853, -5129, -4402, -3672, -2941, -2207, -1472,  -736    
                          };  // equates to 880mVeff at the output


//uint8_t message[128];
// reboot reason is included in e-mail, trapping the brown out etc.
String reset_reason[11] = { 
  "unknown", 
  "power on", 
  "external", 
  "sw restart", 
  "panic", 
  "int watch dog", 
  "task watch dog", 
  "gen watch dog", 
  "deep sleep", 
  "brown out", 
  "over SDIO" };

// *********************************************************************************************
// Some functions
// *********************************************************************************************

//Callback function to get the Email sending status
void sendCallback(SendStatus msg) {
  //Print the current status
  Serial.println(msg.info());
}

// *********************************************************************************************
// Setting up the environment
// *********************************************************************************************
void setup() {
  uint16_t i, j, err = 0;
  String tmp;
  pinMode (R_LED, OUTPUT );
  digitalWrite(R_LED, HIGH);
  pinMode (G_LED, OUTPUT );
  digitalWrite(G_LED, HIGH);
  pinMode (BITX_u, OUTPUT );
  digitalWrite(BITX_u, LOW);
  pinMode (PTT_u, OUTPUT );
  digitalWrite(PTT_u, HIGH);
  pinMode (DEBUG, INPUT_PULLUP);

  analogSetWidth(12);
  analogSetAttenuation(ADC_11db);

  // starting the serial ports
  Wire.begin(I2C_SDA, I2C_SCL, 100000);           // creating a I2C port
  Serial.begin(115200);                           // uart connection used for debugging
  Serial1.begin(4800, SERIAL_8N1, RD_u, TD_u);    // uart connection to/from the µBIT-X
  Serial2.begin(2400, SERIAL_8N1, RD_s, TD_s);    // uart connection to/from the solar controller
  delay(10);
  Serial.println("\nremote, getting awake ...");

  wifiMulti.addAP("SSID_AP_1", "password_AP_1");          // example, to be adapted according to own configuration
  wifiMulti.addAP("SSID_AP_2", "password_AP_2");          // example, to be adapted according to own configuration
  wifiMulti.addAP("SSID_AP_3", "password_AP_3");          // example, to be adapted according to own configuration
  wifiMulti.addAP("SSID_AP_4", "password_AP_4");          // example, to be adapted according to own configuration
    
  //Configures static IP address
  if (!WiFi.config(IP_remote, gateway, subnet, primaryDNS, secondaryDNS))  Serial.println("WiFi failed to configure");
  delay(10);
  if (wifiMulti.run() == WL_CONNECTED) {
    // connected to the Internet: perform NTP, DDNS, DNS and e-mail routines
    Serial.print("connected to WiFi AP: "); Serial.println(WiFi.SSID());
    Serial.print("IP address: ");  Serial.print(WiFi.localIP());
    long rssi = WiFi.RSSI(); // print the received signal strength
    Serial.print("  RSSI: "); Serial.print(rssi); Serial.println(" dBm");
    delay(10);

    // Initialize a NTPClient to get time
    timeClient.begin();   // using UTC in a good ham tradition

    while (!timeClient.update()) {
      Serial.println("Try to get NTP time");
      timeClient.forceUpdate();
      delay(10000);
    }
    
    // The formattedDate comes with the following format:
    // 2018-05-28T16:00:13Z
    // We need to extract date and time
    formattedDate = timeClient.getFormattedDate();

    int splitT = formattedDate.indexOf("T");  // extract date
    dayStamp = formattedDate.substring(0, splitT);
    Serial.print("DATE: ");  Serial.print(dayStamp);
    timeStamp = formattedDate.substring(splitT + 1, formattedDate.length() - 1);
    Serial.print(" HOUR: "); Serial.println(timeStamp);
    delay(1000);

    int hours = timeClient.getHours();
    int minutes = timeClient.getMinutes();

    // synchronise the time of the solar controller
    // must be done within 30s, else the solar contoller will cut our supply
    Serial2.println();
    Serial2.write(0x0d);   delay(1000);
    Serial2.write(0x0d);   delay(1000);
    // setting the time in the solar controller
    Serial2.print('T'); if (hours < 10) Serial2.print('0'); Serial2.print(hours);
    if (minutes < 10) Serial2.print('0'); Serial2.print(minutes); Serial2.write(0x0d); delay(1000);
    Serial2.print('T'); if (hours < 10) Serial2.print('0'); Serial2.print(hours);
    if (minutes < 10) Serial2.print('0'); Serial2.print(minutes); Serial2.write(0x0d); delay(1000);

    // DuckDNS.org allows to update with "http://duckdns.org/update/exampledomain/yourtoken"
    // example, to be adapted according to own configuration  
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
    
    Serial.println("Sending email...");
    //Set the Email host, port, account and password
    // example, to be adapted according to own configuration
    smtpData.setLogin("smtp.provider.com", 587, "your_email@provider.com", "your_email_password");
    //Set the sender name and Email
    smtpData.setSender("your_email_alias", "your_email@provider.com");
    //Set Email priority or importance High, Normal, Low or 1 to 5 (1 is highest)
    smtpData.setPriority("High");
    //Set the subject
    smtpData.setSubject("Remote HF rig daily report");
    //Build the content of the email
    String message = "Hello from the remote HF rig";
     // WiFi connection
     message += "<br>Connected to router \"";
     message += WiFi.SSID();
     message +="\", rssi = ";
     itoa(rssi, tmpstr, 10);  
     message += tmpstr;
     // reset reason
     message += "<br>Awakening from: ";
     esp_reset_reason_t reason = esp_reset_reason();
     message += reset_reason[(int)reason];
     // IP addresses
     message += "<br>Intern IP = ";
     IP_remote = WiFi.localIP();
     message += String(IP_remote[0]);
     message +=".";
     message += String(IP_remote[1]);
     message +=".";
     message += String(IP_remote[2]);
     message +=".";
     message += String(IP_remote[3]);
     message += " Extern IP = ";
     WiFi.hostByName(remote_name, IP_remote);
     message += String(IP_remote[0]);
     message +=".";
     message += String(IP_remote[1]);
     message +=".";
     message += String(IP_remote[2]);
     message +=".";
     message += String(IP_remote[3]);
     message += " Target IP = ";
     if(WiFi.hostByName(home_name, IP_home) != 1) {
      Serial.println("DNS look up failed");
      delay(10000);
      ESP.restart();    // if the DNS look up fails, nothing matters anymore
     }
     message += String(IP_home[0]);
     message +=".";
     message += String(IP_home[1]);
     message +=".";
     message += String(IP_home[2]);
     message +=".";
     message += String(IP_home[3]);
     message += "<br>";
     // flush what ever data is in the serial input buffer
     Serial2.flush();
     while(Serial2.available())Serial2.read();
     // get now some real housekeeping data
     //Serial2.setTimeout(3000);
     Serial2.print("B?"); Serial2.write(0x0d);    delay(1000);
     message += Serial2.readStringUntil(0x0d);    
     message += "<br>";
     Serial2.print("T?"); Serial2.write(0x0d);    delay(1000);
     message += Serial2.readStringUntil(0x0d);    message +=" ";
     Serial2.print("A?"); Serial2.write(0x0d);    delay(1000);
     message += Serial2.readStringUntil(0x0d);    message +=" ";
     Serial2.print("S?"); Serial2.write(0x0d);    delay(1000);
     message += Serial2.readStringUntil(0x0d);
    //Set the message - normal text or html format
    smtpData.setMessage(message, true);
    //Add recipients, can add more than one recipient
    smtpData.addRecipient("your_email@provider.com");
    smtpData.setSendCallback(sendCallback);
    //Start sending Email, can be set callback function to track the status
    if (!MailClient.sendMail(smtpData))  Serial.println("Error sending Email, " + MailClient.smtpErrorReason());
    //Clear all data from Email object to free memory
    smtpData.empty();
    
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

  } else {      
    // if unsccussfull to connect to a WiFi Access point, become one 
    // no NTP and e-mail possibilities
    WiFi.disconnect();
    delay(200);
    WiFi.mode(WIFI_AP);
    delay(200);
    WiFi.softAP(AP_ssid, AP_password);
    delay(200);
    if(!WiFi.softAPConfig(IP_remote, IP_remote, subnet)) {
    Serial.println("AP Config Failed");      
      while (1) {     // endless loop, no sense to go further
        delay(10);
      }
    }
    Serial.print("no WiFi AP found, making own: \"RemoteHF\""); 
    // fake the time for the solar controller
    // must be done within 30s, else the solar contoller will cut the supply
    int hours = 20;  int minutes = 0;  // let's pretend it is allways 20:00 
    Serial2.println();
    Serial2.write(0x0d);   delay(1000);
    Serial2.write(0x0d);   delay(1000);
    // setting the time in the solar controller
    Serial2.print('T'); if (hours < 10) Serial2.print('0'); Serial2.print(hours);
    if (minutes < 10) Serial2.print('0'); Serial2.print(minutes); Serial2.write(0x0d); delay(1000);
    Serial2.print('T'); if (hours < 10) Serial2.print('0'); Serial2.print(hours);
    if (minutes < 10) Serial2.print('0'); Serial2.print(minutes); Serial2.write(0x0d); delay(1000);
  }

  // pre fill the sound buffer with 1s of a chirping sine wave of 300...1200Hz
  accu = 0;
  adder = 2400;
  for ( i = 0; i < snd_buf_size; i++) {
    accu += adder;
    j = accu >> 8;
    snd_buf[i] = sine[j];
    if ( i % 2 ) adder++;
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
  if (err == 0) Serial.println("codec initialized");
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

  // ending the setup
  Serial.print("ready! use ");  Serial.println(WiFi.localIP());
  udp.begin(udp_r_port);      // start listing to incoming udp packets
  delay(10);
  digitalWrite(R_LED, LOW);
  digitalWrite(G_LED, LOW);
  connect_time = millis() - no_con_time + 100;

}  // end setup

// ********************************************************************************************
// Dedicated audio task, this is in effect a 2nd "loop"
// ********************************************************************************************
void audioTask(void *) {
  size_t byte_num;
  Serial.println("dedicated audio task created");

  while (true) {      // audio task will never end

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
    if (snd_buf_fill > (snd_buf_size / 2)) {
      snd_on = true;
    }
    if (snd_buf_fill < (snd_buf_size / 16)) {
      snd_on = false;
    }

  // check if I2S output DMA buffer needs a refill
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

  //  ************************* handling an incomming UDP packet  ********************************
  // if there's data available, read a packet
  int packetSize = udp.parsePacket();
  if (packetSize == udp_buf_size) {
    udp.read(udp_r_buf, udp_buf_size);
    rx_pac_cnt++;

    // handle sound part
    for (i = 0; i < in_buf_size; i++) {
      // transform the first 640 bytes into 320 sound samples
      snd_buf[snd_buf_w] = (uint16_t)udp_r_buf[i * 2] << 8;
      snd_buf[snd_buf_w] += (uint16_t)udp_r_buf[(i * 2) + 1];
      snd_buf_w++;
      if (snd_buf_w == snd_buf_size) snd_buf_w = 0;
    }

    // decode the count byte
    pac_num = (uint16_t)udp_r_buf[udp_pac_cnt];
    ctl_cnt = (uint16_t)(udp_r_buf[udp_ctl_cnt] >> 4);
    if ((udp_r_buf[udp_ctl_cnt] & 0x08) == 0x08) ptt_rf = true;
    else ptt_rf = false;
    cat_cnt = (uint16_t)(udp_r_buf[udp_ctl_cnt] & 0x07);

    // handle the CAT part
    if ((ptt_rf == true) && (ptt_rf_old == false)) ptt_tim = millis();
    if ((ptt_rf == false) && (ptt_rf_old == true)) ptt_tim = millis();  
    ptt_rf_old = ptt_rf; 
    if ((millis() - ptt_tim) > ptt_del) {
      if (ptt_rf == true) digitalWrite(PTT_u, HIGH);  // propagate the PTT signal
      else digitalWrite(PTT_u, LOW);                  // after a short while
    }

    if (cat_cnt != 0) {
      for (j = 0; j < cat_cnt; j++) {
        Serial1.write(udp_r_buf[udp_cat_mes + j]);
      }
    }

    // handle the solar Controller part
    if (ctl_cnt != 0) {
      for (j = 0; j < ctl_cnt; j++) {
        Serial2.write(udp_r_buf[udp_ctl_mes + j]);
      }
    }   

  digitalWrite(G_LED, !digitalRead(G_LED));
  connect_time = millis();
  BITX_on = true;

  }
  delay(1);
  
  //  ************************* handling an outgoing UDP packet  ********************************
  // check if we're ready to send a buffer of samples over wifi
  if (current_out_buf != current_in_buf) {
    for (i = 0; i < in_buf_size; i++) {
      udp_s_buf[i * 2] = (uint8_t)(in_buf[current_out_buf][i] >> 8);
      udp_s_buf[(i * 2) + 1] = (uint8_t)in_buf[current_out_buf][i];
    }

    udp_s_buf[udp_pac_cnt] = (uint8_t)tx_pac_cnt;  // use position 1280/640 to count the packets 0...255
    udp_s_buf[udp_ctl_cnt] = 0x00; 
    
    // append with the data received from the CAT
    udp_s_buf[udp_ctl_cnt] = 0x00;   // position 1281/641 is the actual CAT & CTL byte count
    if (i = Serial1.available()) { // 5 chars max
      if (i > 5) i = 5;   // limit the length
      Serial1.readBytes(&udp_s_buf[udp_cat_mes], i);  // hopefully at the right place
      udp_s_buf[udp_ctl_cnt] += (char)i;
    }

    // check the internal ctl processes
    // timing is performed by counting TX packets, should be 25/s
    // care is taken to have only one message per packet

    // forward & reflected power is evaluated every 80ms, a running average of 1/16
    // shoud equate to a LPF of about 2Hz, hopefully enough to eliminate the noise
    fwd_pow = fwd_pow + analogRead(SWR_fwd) - (fwd_pow >> 4);
    rfl_pow = rfl_pow + analogRead(SWR_rfl) - (rfl_pow >> 4);

    if ((tx_pac_cnt % 16) == 3) {   // power reading sent every 0.7s or so
      sprintf(tmpstr, "P=%d/%d\r", (fwd_pow >> 4), (rfl_pow >> 4));
      i = 0;
      while (tmpstr[i] != 0) {
        udp_s_buf[udp_ctl_mes + i] = tmpstr[i];
        i++;
        if (i == 13) break;
      }
      udp_s_buf[udp_ctl_cnt] |= (char)(i << 4) & 0xf0;
      //Serial.println(tmpstr);  // debugging
    }

    if ((tx_pac_cnt % 1024) == 4) {   // received count report sent every 41s or so
      sprintf(tmpstr, "Z=%d\r", rx_pac_cnt);
      i = 0;
      while (tmpstr[i] != 0) {
        udp_s_buf[udp_ctl_mes + i] = tmpstr[i];
        i++;
        if (i == 13) break;
      }
      udp_s_buf[udp_ctl_cnt] |= (char)(i << 4) & 0xf0;
      rx_pac_cnt = 0;
    }

    if ((tx_pac_cnt % 256) == 5) {   // WiFi RSSI value sent every 10s or so
      sprintf(tmpstr, "Y=%d\r", (int)WiFi.RSSI());
      i = 0;
      while (tmpstr[i] != 0) {
        udp_s_buf[udp_ctl_mes + i] = tmpstr[i];
        i++;
        if (i == 13) break;
      }
      udp_s_buf[udp_ctl_cnt] |= (char)(i << 4) & 0xf0;
    }

    if ((tx_pac_cnt % 256) == 6) {   // get a battery report sent every 10s or so
      Serial2.print("B?\r");
    }

    // append with the data received from the Solar controller
    if ((udp_s_buf[udp_ctl_cnt] & 0xf0) == 0) {  // if not enough room, skip this packet and take the next
      // buffer the input from the solar-controller until a "\r" char has been received
      if (i = Serial2.available()) { 
        for (j = 0; j < i; j++) {
          in_sol = Serial2.read();
          if (in_sol == '\r') in_sol_fl = true;         // end of message was reached
          if (in_sol > 0x1f ) {                         // only printable chars are stored
            in_sol_buf[in_sol_pnt] = in_sol;         
            in_sol_pnt++;
            if (in_sol_pnt == 16) in_sol_pnt--;         // do not exceed the buffer size 
          }
        }
      }
      // if a message was received put it on the packet train
      if (in_sol_fl) {
        if (in_sol_pnt > 13) in_sol_pnt = 13;           // limit the length to 13 chars max
        for (i = 0; i < in_sol_pnt; i++) udp_s_buf[udp_ctl_mes + i] = in_sol_buf[i];
        udp_s_buf[udp_ctl_cnt] |= (char)(in_sol_pnt << 4) & 0xf0;
        in_sol_pnt = 0;
        in_sol_fl = false;
      }
   }
    if (ptt_rf) udp_s_buf[udp_ctl_cnt] |= 0x08;         // also echo the PTT status on the packet train
  
    // actual sending if send flag is true
    if (BITX_on) {
      // send everything in a single packet
      udp.beginPacket(IP_home, udp_h_port);
      udp.write(udp_s_buf, udp_buf_size);
      udp.endPacket();
      tx_pac_cnt++;       
      digitalWrite(R_LED, !digitalRead(R_LED));
    }
    current_out_buf = current_in_buf;

  }
  delay(1);

// ******************** handling other stuff ***********************************

  // check if the connection was lost
  if(millis() - connect_time > no_con_time) {  // no packets received for 10s
    BITX_on = false; 
    connect_time = millis() - no_con_time + 100;
    tx_pac_cnt = 0;
    rx_pac_cnt = 0; 
  }
  if ((old_BITX_on) && (!BITX_on)) {
    digitalWrite(BITX_u, LOW);
    Serial.println("switching off");
  }
  if ((!old_BITX_on) && (BITX_on)) {
    digitalWrite(BITX_u, HIGH);
    Serial.println("switching on");
  }
  old_BITX_on = BITX_on;
  
  // check the OTA update server incoming request
  server.handleClient();

  delay(1);
}  // end loop
