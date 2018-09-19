#include <MIDI.h>
#include <SPI.h>

//The MIDI channel to listen on. Set to 0 for "all channels"
#define MIDI_CHANNEL 1

//Uncomment this to get debug output on serial console. May add some latency to MIDI output
//#define DEBUG_PRINT

#define UNUSED_CC 255
#define HIRES_CC_AUTO 254

//MIDI CC messages used for the different outputs 0-2 (pitch bend is #3)
uint8_t cc_list[4] = {
  2,        //Breath
  4,        //Foot pedal
  5,        //Porta time
  UNUSED_CC //Unused
};

//The output position of the pitch bend. If defined, it will use pitch bend instead of
//the fourth CC slot
#define PITCH_BEND


//Chip select pins for SPI bus to DACs. Any available digital output works. The board
//default pins are used for MOSI and SCK.
uint8_t cs_pins[2] = {5, 7};


//High-resolution CC messages. Typically "regular CC" + 32. Can be manually set here.
//If set to HIRES_CC_AUTO, these are automatically assigned to CC+32 where appropriate.
//If hi-res is unwanted, set this to UNUSED_CC to block it off
uint8_t cc_hires[4] = {
  HIRES_CC_AUTO,
  HIRES_CC_AUTO,
  HIRES_CC_AUTO,
  UNUSED_CC
};

//Flags to set high=1 (0-4V) or low=0 gain (0-2V) per DAC.
uint8_t dac_gain[4] = {0, 0, 0, 0};


//Flags to invert channel. Normal operation (0) is 0V at 0 CC value, 1 inverts this.
uint8_t cv_invert[4] = {0, 0, 0, 0};


//Do midi with default settings.
//MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);
MIDI_CREATE_DEFAULT_INSTANCE();

//High and low bytes of each CC. Pitch bend initially set to center value, the rest are 0.
uint8_t cc_msb[4];
uint8_t cc_lsb[4];


//Keep track of wether or not we have seen a hi-res message for this CC.
bool hires_seen[4] = {false, false, false, false};

//Define number of CC slots, and which slot pitch bend goes into
#ifdef PITCH_BEND
#define N_CC 3
#else
#define N_CC 4
#endif

#define PITCH_BEND_SLOT 3


void setup() {

#ifdef DEBUG_PRINT
  Serial.begin(115200);
  delay(2000); //Let serial console catch up to reality after reboot
#endif

  //Initialise outputs
  SPI.begin();
  for(uint8_t i=0; i<2; ++i) {
    pinMode(cs_pins[i], OUTPUT);
    digitalWrite(cs_pins[i], HIGH);
  }
  for(uint8_t i=0; i<4; ++i) {
    cc_msb[i] = 0;
    cc_lsb[i] = 0;

#ifdef PITCH_BEND
    if(i==PITCH_BEND_SLOT) {
      cc_msb[i] = 0x40;       //Set pitch bend initial value at center position
      cc_lsb[i] = 0x00;
    }
#endif

    sendOutput(i);
  }

  //Setup high-resolution CC
  for(uint8_t i=0; i<4; ++i) {
    if(cc_hires[i] == HIRES_CC_AUTO) {
      uint8_t main_cc = cc_list[i];
      if(main_cc<=31) {
        cc_hires[i] = main_cc+32;
      } else {
        //Set to auto but cannot be used
        cc_hires[i] == UNUSED_CC;
      }
    }
  }

  // Initialise MIDI. Set everything to go through.
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.setThruFilterMode(midi::Thru::Full);

  MIDI.setHandleControlChange(midiChangeControl);

#ifdef PITCH_BEND
  MIDI.setHandlePitchBend(midiPitchBend);
#endif

#ifdef DEBUG_PRINT
  Serial.println("TurbineCV ready");
#endif
}

//Just sit here polling for MIDI data. The event handlers do the work.
void loop() {
  MIDI.read();
}



//Callback for CC messages
void midiChangeControl(byte channel, byte number, byte value) {

#ifdef DEBUG_PRINT
  Serial.print("C: ");
  Serial.print(channel);
  Serial.print(" cc: ");
  Serial.print(number);
  Serial.print(" v: ");
  Serial.println(value);
#endif

  if(MIDI_CHANNEL && channel != MIDI_CHANNEL) return; //Only care about specific channel

  for(uint8_t i=0; i<N_CC; ++i) {
    if(number==cc_list[i]) {
      //Read MSB ("low-res" part)
      if(value != cc_msb[i]) {
        if(hires_seen[i]) {
          //There has been a high-res (MSB) value seen since last time the LSB was set.
          //Assume there will be another one soon, so do not send output yet. Typically
          //controllers will send this first and the high-res one right after.
          cc_msb[i] = value;
          hires_seen[i] = false; //Clear flag to check next interval
        } else {
          //There was no high-res value since last regular one. Assume there is no
          //high-res data sent. The LSB part is set to the same as MSB, to linearly scale
          //values over the entire DAC range
          cc_lsb[i] = cc_msb[i] = value;

#ifdef DEBUG_PRINT
          Serial.print("Sending slot ");
          Serial.print(i);
          Serial.println(" (low-res)");
#endif

          sendOutput(i);  //Send data now, as this is likely all we are getting
        }
      }
      // return; //allow for same CC to be used on multiple outputs
    }

    if(number==cc_list[i]+32) {
      //Read LSB ("high-res" part), and send it right away.
      cc_lsb[i] = value;
      hires_seen[i] = true;
#ifdef DEBUG_PRINT
      Serial.print("Sending slot ");
      Serial.print(i);
      Serial.println(" (high-res)");
#endif
      sendOutput(i);
      // return;
    }

  }
}


//Callback for pitch bend messages
void midiPitchBend(byte channel, int bend) {

#ifdef DEBUG_PRINT
  Serial.print("C: ");
  Serial.print(channel);
  Serial.print(" pb: ");
  Serial.println(bend);
#endif

  if(MIDI_CHANNEL && channel != MIDI_CHANNEL) return; //Only care about specific channel

  //Midi pitch bend value is signed, -8192 - 8191. Translate it to 0-16383
  bend += 8192;

  //Split it up into two 7-bit parts to look like a hi-res cc message
  cc_msb[PITCH_BEND_SLOT] = bend >> 7;
  cc_lsb[PITCH_BEND_SLOT] = bend & 0x7F;
  sendOutput(PITCH_BEND_SLOT);
}


void sendOutput(byte slot) {
  uint8_t pin = cs_pins[slot>>1]; //map cc position to DAC chip (CS pin)
  uint8_t dac = slot % 2;            //map cc position to DAC unit in chip

  //Make two 7-bit parts into one 12-bit value
  uint16_t value = ((cc_msb[slot] << 5) | (cc_lsb[slot] >> 2)) & 0X0FFF;

  if(cv_invert[slot]) value = 0x0FFF - value;

  uint16_t command = (dac?0x9000:0x1000); //DAC select + not shutdown
  if(!dac_gain[slot]) command |= 0x2000;  //Set "low gain" bit
  command |= value;

#ifdef DEBUG_PRINT
  Serial.print("Send ");
  Serial.print(value);
  Serial.print(" cs ");
  Serial.print(pin);
  Serial.print(" dac ");
  Serial.print(dac);
  Serial.print(" command ");
  Serial.println(command, HEX);
#endif

  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(pin, LOW);
  //Transfer command bytes separately
  SPI.transfer(command >> 8);
  SPI.transfer(command & 0x00FF);
  digitalWrite(pin, HIGH);
  SPI.endTransaction();
}
