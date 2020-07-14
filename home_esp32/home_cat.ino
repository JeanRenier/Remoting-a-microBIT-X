// To cope with the rather stringent timing requirements of the applications requiring CAT 
// which precludes a transparant link between the remote and the home devices, a two fold approach has been made.
// The home device includes a full FT-817/FT-857 emulation, which will immedialely will resond to the commands
// When the frequency or the mode is changed (the only things that can actually be programmed in the µBIT-X) 
// a typical 5 byte FT-817 command is send by the home to the remote, "0x01" & "0x07" commands. 
// frequency/mode read request ("0x03" commands) are send every 640ms, the response by the µBIT-X is eventually 
// received by the home and is rendered in the 1st line of the OLED.  

// Some of this was borrowed from the µBIT-X sofware.

// **************  defines associated with the CAT, only a few are implemented to keep the applications on the PC happy
// >>> First single command ones (toggle)
//~ #define CAT_LOCK_ON             0x00
//~ #define CAT_LOCK_OFF            0x80
#define CAT_PTT_ON              0x08
#define CAT_PTT_OFF             0x88
//~ #define CAT_CLAR_ON             0x05
//~ #define CAT_CLAR_OFF            0x85
//~ #define CAT_SPLIT_ON            0x02
//~ #define CAT_SPLIT_OFF           0x82
#define CAT_VFO_AB              0x81
// >>> Now complex ones
#define CAT_FREQ_SET            0x01
#define CAT_MODE_SET            0x07
//~ #define CAT_CLAR_SET            0xF5
#define CAT_RX_DATA_CMD         0xE7
#define CAT_TX_DATA_CMD         0xF7
#define CAT_RX_FREQ_CMD         0x03
//~ #define CAT_RPTR_OFFSET_CMD     0x09
//~ #define CAT_RPTR_FREQ_SET       0xF9
//~ #define CAT_SQL_CMD             0x0A
// >>> Modes definition
//~ #define CAT_MODE_LSB            0x00
//~ #define CAT_MODE_USB            0x01
//~ #define CAT_MODE_CW             0x02
//~ #define CAT_MODE_CWR            0x03
//~ #define CAT_MODE_AM             0x04
//~ #define CAT_MODE_FM             0x08
//~ #define CAT_MODE_DIG            0x0A
//~ #define CAT_MODE_PKT            0x0C
//~ #define CAT_MODE_FMN            0x88
// >>> SQL modes
//~ #define CAT_SQL_DCS             0x0A
//~ #define CAT_SQL_DCS_DECD        0x0B
//~ #define CAT_SQL_DCS_ENCD        0x0C
//~ #define CAT_SQL_CTCSS           0x2A
//~ #define CAT_SQL_CTCSS_DECD      0x3A
//~ #define CAT_SQL_CTCSS_ENCD      0x4A
//~ #define CAT_SQL_OFF             0x8A
//~ #define CAT_SQL_CTCSS_SET       0x0B
//~ #define CAT_SQL_DCS_SET         0x0C
// >>> RPT related
//~ #define CAT_RPTR_OFFSET_N       0x09
//~ #define CAT_RPTR_OFFSET_P       0x49
//~ #define CAT_RPTR_OFFSET_S       0x89
// >>> HAMLIB specific ones
#define CAT_HAMLIB_EEPROM       0xBB

//  ******* Periodic call function, this must be called often from inside the loop()
void CATcheck() {
    // do nothing if it was disabled by software
    if (!cat_enabled) return;

    // first check if we have at least 5 bytes waiting on the buffer
    byte i = Serial1.available();
    if (i < 5) return;

    // if you got here then there is at least 5 bytes waiting: get it.
    for (i=0; i<5; i++) {
      nullPad[i] = Serial1.read();
    }
    //Serial.print("i= "); 
    //for (byte i=0; i<5; i++) {
    //    if (nullPad[i] < 16) Serial.print("0"); 
    //    Serial.print(nullPad[i], HEX);
    //    Serial.print(" ");
    //}
    //Serial.println();
       
    // now check for the command in the last byte
    switch (nullPad[4]) {
        case CAT_PTT_ON:
          ptt_cat = true;
          nullPad[0] = 0x00;
          sent(1);  
          break;
        case CAT_PTT_OFF:
          ptt_cat = false;
          nullPad[0] = 0x00;
          sent(1);  
          break;
        case CAT_VFO_AB:
          nullPad[0] = 0x00;
          sent(1);  
          break;
        case CAT_FREQ_SET:
          fset();
          nullPad[0] = 0x00;
          sent(1);  
          break;
        case CAT_MODE_SET:
          cat_mode = nullPad[0];
          nullPad[0] = 0x00;
          sent(1);  
          break;
        case CAT_RX_FREQ_CMD:
          sendFreqMode(); // without ACK
           break;
        case CAT_HAMLIB_EEPROM:
          readEeprom();
          break;
        case CAT_RX_DATA_CMD:
          rxStatus(); // without ACK
          break;
        case CAT_TX_DATA_CMD:
          sendTxStatus(); // without ACK
            break;
        default:
          nullPad[0] = 0x00;
          sent(1);
          break;
    }
}

// set a frequency
void fset() {
    // reconstruct the freq from the bytes we got
    from_bcd_be();
}

// send the TX status
void sendTxStatus() {
    // just one byte with the format the CAT expect, see the example in the library
    nullPad[0] = 0x00;
    sent(1);
}

// send freq and mode
void sendFreqMode() {
    // this function must return 5 bytes via the serial port, the first four
    // are the freq in BCD BE and the 5th is the mode   
    npadClear(); // clear the nullpad
    writeFreq(freq, nullPad); // put the freq in the nullPad 4 first bytes
    //to_bcd_be(freq);        
    nullPad[4] = cat_mode;    // put the mode in the last byte
    sent(5);
}

// READ EEPROM, this is a trick of Hamlib
void readEeprom() {
    // This is to make hamlib happy, PC requested reading two bytes
    // we must answer with two bytes, we forge it as empty ones or...
    // if the second byte in the request is 0x78 we have to send the first
    // with the 5th bit set if the USB or zero if LSB.
    byte temp = nullPad[1]; // mem zone to "read"
    npadClear();        // clear the nullpad
    sent(2);
}

// read the rx Status
void rxStatus() {
    /*
     * Data to be returned
     *    D1 = {0xij} i = 0 = squelch off
     *                i = 1 = squelch on
     *                j = 0 = CTCSS/DCS matched
     *                j = 1 = CTCSS/DCS unmatched
     *    D2 = {0xkl} k = 0 = discriminator centered
     *                k = 1 = discriminator offcentered
     *                l = dummy data
     *    D3-D4 = S-meter data
     *
    */
    // clear the nullpad
    npadClear();
    // we only return the s-meter here, just the 4 bits.
    nullPad[0] = 0b00001111;
    sent(1);
}

// procedure to clear the nullpad
void npadClear() {
    // this is used to initialize the nullpad
    for (byte i=0; i<5; i++) nullPad[i] = 0;
}

// sent the data to the PC
void sent(byte amount) {
   // sent the nullpad content
   //Serial.print("o= "); 
   for (byte i=0; i<amount; i++) {
      Serial1.write(nullPad[i]);
      //if (nullPad[i] < 16) Serial.print("0"); 
      //Serial.print(nullPad[i], HEX);
      //Serial.print(" ");
   }
  //Serial.println();
}
// put the freq in the nullpad array
void to_bcd_be(long f) {
    unsigned char a;

    // the freq is sent the 10th of the hz
    f /= 100;

    // clear the nullpad
    npadClear();

    // do the magic
    nullPad[3] &= 0x0f;
    nullPad[3] |= (f%10)<<4;
    f /= 10;

    for (int i=2; i >= 0; i--) {
        a = f%10;
        f /= 10;
        a |= (f%10)<<4;
        f /= 10;
        nullPad[i] = a;
    }
}

// put the freq in the freq var from the nullpad array
void from_bcd_be() {
    // e.g. {0x01,0x40,0x07,0x00,0x01} tunes to 14.070MHz
    freq = 0;
    for (byte i=0; i<4; i++) {
        freq *= 10;
        freq += nullPad[i]>>4;
        freq *= 10;
        freq += nullPad[i] & 0x0f;
    }

    freq *= 10;
    freq += nullPad[4]>>4;
}


// ******************* other stuff     *****************
void  catFreqUpdate(void){    // request a frequency update
    cat_out[0] = 0x00;                     
    cat_out[1] = 0x00;
    cat_out[2] = 0x00;
    cat_out[3] = 0x00;
    cat_out[4] = 0x03;
    cat_out_fl = true; 
}

// This function takes a frquency that is encoded using 4 bytes of BCD
// representation and turns it into an long measured in Hz. e.g. [01][41][23][45] = 14.12345 Mhz
unsigned long readFreq(byte* cmd) {
    // Pull off each of the digits
    byte d7 = getHighNibble(cmd[0]);
    byte d6 = getLowNibble(cmd[0]);
    byte d5 = getHighNibble(cmd[1]);
    byte d4 = getLowNibble(cmd[1]); 
    byte d3 = getHighNibble(cmd[2]);
    byte d2 = getLowNibble(cmd[2]); 
    byte d1 = getHighNibble(cmd[3]);
    byte d0 = getLowNibble(cmd[3]); 
    return  
      (unsigned long)d7 * 100000000L +
      (unsigned long)d6 * 10000000L +
      (unsigned long)d5 * 1000000L + 
      (unsigned long)d4 * 100000L + 
      (unsigned long)d3 * 10000L + 
      (unsigned long)d2 * 1000L + 
      (unsigned long)d1 * 100L + 
      (unsigned long)d0 * 10L; 
}

// Takes a frequency and writes it into the CAT command buffer in BCD form.
void writeFreq(unsigned long freq, byte* cmd) {
  // Convert the frequency to a set of decimal digits. We are taking 9 digits
  // so that we can get up to 999 MHz. But the protocol doesn't care about the
  // LSD (1's place), so we ignore that digit.
  byte digits[9];
  getDecimalDigits(freq,digits,9);
  // Start from the LSB and get each nibble 
  cmd[3] = setLowNibble(cmd[3],digits[1]);
  cmd[3] = setHighNibble(cmd[3],digits[2]);
  cmd[2] = setLowNibble(cmd[2],digits[3]);
  cmd[2] = setHighNibble(cmd[2],digits[4]);
  cmd[1] = setLowNibble(cmd[1],digits[5]);
  cmd[1] = setHighNibble(cmd[1],digits[6]);
  cmd[0] = setLowNibble(cmd[0],digits[7]);
  cmd[0] = setHighNibble(cmd[0],digits[8]);  
}

byte setHighNibble(byte b,byte v) {
  // Clear the high nibble
  b &= 0x0f;
  // Set the high nibble
  return b | ((v & 0x0f) << 4);
}

byte setLowNibble(byte b,byte v) {
  // Clear the low nibble
  b &= 0xf0;
  // Set the low nibble
  return b | (v & 0x0f);
}

byte getHighNibble(byte b) {
  return (b >> 4) & 0x0f;
}

byte getLowNibble(byte b) {
  return b & 0x0f;
}

// Takes a number and produces the requested number of decimal digits, starting from the least significant digit.  
void getDecimalDigits(unsigned long number,byte* result,int digits) {
  for (int i = 0; i < digits; i++) {
    // "Mask off" (in a decimal sense) the LSD and return it
    result[i] = number % 10;
    // "Shift right" (in a decimal sense)
    number /= 10;
  }
}
