#include <MIDI.h>
#include <SPI.h>

#include "notes.h"
#include "cvtable.h"

//The MIDI channel to listen on. Set to 0 for "all channels"
#define MIDI_CHANNEL 0

//Pitch bend range, in number of half tones (12 = one octave)
#define PITCH_BEND_RANGE 5

//Uncomment this to get debug output on serial console. May add some latency to MIDI output. Only do this on
//arduinos where serial console does not use the same hardware serial port as MIDI
#define DEBUG_PRINT

#define UNUSED_CC 255
#define HIRES_CC_AUTO 254

#define N_DAC 6


//MIDI CC messages used for the different outputs
uint8_t cc_list[N_DAC] = {
  UNUSED_CC,  //Note
  5,          //Glide (portamento time)
  UNUSED_CC,  //Pitch bend
  2,          //Breath
  1,          //Mod wheel
  UNUSED_CC   //Velocity
};

//"DAC slots" where note pitch (v/oct) and velocity are sent. DAC 0 is slots 0 and 1, DAC 1 is 2 and 3, DAC 2 is 4 and 5.
#define NOTE_SLOT 0
#define BEND_SLOT 2
#define VELOCITY_SLOT 5

#define AT_SLOT 3 //Aftertouch goes in breath slot

//High-resolution CC messages. Typically "regular CC" + 32. Can be manually set here.
//If set to HIRES_CC_AUTO, these are automatically assigned to CC+32 (where main CC is in 1-31 range).
//If hi-res is unwanted, set this to UNUSED_CC to block it off
//Slots that are not used for CC's are disabled
uint8_t cc_hires[N_DAC] = {
  UNUSED_CC,
  HIRES_CC_AUTO,
  UNUSED_CC,
  HIRES_CC_AUTO,
  HIRES_CC_AUTO,
  UNUSED_CC,
};

//Digital output pins for gate and trigger
#define TRIG_PIN 5
#define GATE_PIN 6

//Digital output pin for activity LED
#define LED_PIN 10

//Input pins for switches
#define GLIDE_SWITCH          7 //SW2
#define BEND_SWITCH           8 //SW3
#define AFTERTOUCH_SWITCH     9 //SW4

//How long to run the trigger pulse for (roughly) in microseconds
#define TRIGGER_WIDTH 100

//How long to turn the activity LED on for, in milliseconds
#define MIDI_ACT_TIME 50

//How often to read onboard switches
#define SWITCH_READ_INTERVAL 500

//Chip select pins for SPI bus to DACs. The board default pins are used for MOSI and SCK.
uint8_t cs_pins[N_DAC/2] = {2, 3, 4};

//Flags to set high=1 (0-4V) or low=0 gain (0-2V) per DAC.
uint8_t dac_gain[N_DAC] = {1, 0, 0, 0, 0, 0};

//Flags to invert DAC channel. Normal operation (0) is lowest voltage at 0 CC value / low note, 1 inverts this.
uint8_t cv_invert[N_DAC] = {0, 0, 0, 0, 0, 0};


//Map two 7-bit parts into one 12-bit value
#define map14to12(msb, lsb)  ((((uint16_t)msb << 5) | (lsb >> 2)) & 0X0FFF)


bool trigger_on = false;
int8_t current_note = NO_NOTE; //MIDI value of currently playing note
uint8_t note_velocity = 0; //Velocity of currently playing note
unsigned long trigger_on_time;
int pitch_bend = 0; //Signed value, -8192 - +8191


bool glide_merge;
bool bend_merge;
bool use_aftertouch;
unsigned long switchesRead;


//Do midi with default settings.
//MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);
MIDI_CREATE_DEFAULT_INSTANCE();

//High and low bytes of each CC.
uint8_t cc_msb[N_DAC];
uint8_t cc_lsb[N_DAC];


//Keep track of wether or not we have seen a hi-res message for this CC.
bool hires_seen[N_DAC] = {false, false, false, false, false, false};

void setup() {

#ifdef DEBUG_PRINT
  Serial.begin(115200);
  delay(2000); //Let serial console catch up to reality after reboot
#endif

  note_init();

  //Initialise in- and outputs
  pinMode(GATE_PIN, OUTPUT);
  digitalWrite(GATE_PIN, LOW);
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(GLIDE_SWITCH, INPUT_PULLUP);
  pinMode(BEND_SWITCH, INPUT_PULLUP);
  pinMode(AFTERTOUCH_SWITCH, INPUT_PULLUP);

  readSwitches();
  
  SPI.begin();
  for(uint8_t i=0; i<N_DAC/2; ++i) {
    pinMode(cs_pins[i], OUTPUT);
    digitalWrite(cs_pins[i], HIGH);
  }
  for(uint8_t i=0; i<N_DAC; ++i) {
    cc_msb[i] = 0;
    cc_lsb[i] = 0;
    if(cc_list[i] != UNUSED_CC) {
      sendCC(i);
    }
  }

  //Setup high-resolution CC
  for(uint8_t i=0; i<N_DAC; ++i) {
    if(cc_hires[i] == HIRES_CC_AUTO) {
      uint8_t main_cc = cc_list[i];
      if(main_cc >= 1 && main_cc<=31) {
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

  //Handle MIDI things
  MIDI.setHandleControlChange(midiChangeControl);
  MIDI.setHandlePitchBend(midiPitchBend);
  MIDI.setHandleNoteOn(midiNoteOn);
  MIDI.setHandleNoteOff(midiNoteOff);
  MIDI.setHandleAfterTouchChannel(midiAftertouchChannel);

#ifdef DEBUG_PRINT
  Serial.println("TurbineCV ready");
#endif
}


void loop() {

  //Poll for MIDI data. This will trigger handler functions if anything comes in.
  MIDI.read();

  //See if it's time to turn trigger off
  if(trigger_on){
    if(micros() >= trigger_on_time + TRIGGER_WIDTH) {
      digitalWrite(TRIG_PIN, LOW);
      trigger_on = false;
    }
  }

  if(millis() >= switchesRead + SWITCH_READ_INTERVAL) {
    readSwitches();
  }
}

//Callback for CC messages
void midiChangeControl(byte channel, byte number, byte value) {

#ifdef DEBUG_PRINT_CC
  Serial.print("Chan: ");
  Serial.print(channel);
  Serial.print(" cc: ");
  Serial.print(number);
  Serial.print(" v: ");
  Serial.println(value);
#endif

  if(MIDI_CHANNEL && channel != MIDI_CHANNEL) return; //Only care about specific channel

  for(uint8_t i=0; i<N_DAC; ++i) {
    if(number==cc_list[i]) {

      //Ignore this value if aftertouch is used for this slot
      if(use_aftertouch && i==AT_SLOT) {
        return;
      }

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

#ifdef DEBUG_PRINT_CC
          Serial.print("Sending slot ");
          Serial.print(i);
          Serial.println(" (low-res)");
#endif

          sendCC(i);  //Send data now, as this is likely all we are getting
        }
      }
      // return; //allow for same CC to be used on multiple outputs for whatever reason
    }

    if(number==cc_list[i]+32) {
      //Read LSB ("high-res" part), and send it right away.
      cc_lsb[i] = value;
      hires_seen[i] = true;

#ifdef DEBUG_PRINT_CC
      Serial.print("Sending slot ");
      Serial.print(i);
      Serial.println(" (high-res)");
#endif
      sendCC(i);
      // return;
    }

  }
}


//Callback for pitch bend messages
void midiPitchBend(byte channel, int bend) {

#ifdef DEBUG_PRINT
  Serial.print("Chan: ");
  Serial.print(channel);
  Serial.print(" pb: ");
  Serial.println(bend);
#endif

  if(MIDI_CHANNEL && channel != MIDI_CHANNEL) return; //Only care about specific channel

  int pb_dac = (bend+8192) / 4; //Scale pb value from -8192 - 8191 down to "dac range", 0-4095
  sendDac(BEND_SLOT, pb_dac);
  pitch_bend = bend;

  if(bend_merge) {
    play_note(false);  
  }
}

void midiNoteOn(byte channel, byte note, byte velocity) {

  #ifdef DEBUG_PRINT
  Serial.print("Chan: ");
  Serial.print(channel);
  Serial.print(" note on: ");
  Serial.print(note);
  Serial.print(" vel: ");
  Serial.println(velocity);
  #endif

  if(MIDI_CHANNEL && channel != MIDI_CHANNEL) return;

  //Just ignore notes outside of range (these could be "folded back in")
  if(note < MIDI_NOTE_LOW || note > MIDI_NOTE_HIGH) return;

  //current_note = note_on(note);
  current_note = note;
  note_velocity = velocity;

  play_note(true);

}


void midiNoteOff(byte channel, byte note, byte velocity) {

  #ifdef DEBUG_PRINT
  Serial.print("Chan: ");
  Serial.print(channel);
  Serial.print(" note off: ");
  Serial.print(note);
  Serial.print(" vel: ");
  Serial.println(velocity);
  #endif

  if(MIDI_CHANNEL && channel != MIDI_CHANNEL) return;

  if(note < MIDI_NOTE_LOW || note > MIDI_NOTE_HIGH) return;

  //Leave last note playing, just mute it
  //current_note = note_off(note);
  //current_note = NO_NOTE;
  if(note != current_note) return;

  note_velocity = 0;
  digitalWrite(GATE_PIN, LOW);
  sendVelocity(note_velocity);

}

void midiAftertouchChannel(byte channel, byte pressure) {

  #ifdef DEBUG_PRINT
  Serial.print("Chan: ");
  Serial.print(channel);
  Serial.print(" aftertouch: ");
  Serial.println(pressure);
  #endif
  
  //Bail out if not set to use aftertouch (via switch)
  if(!use_aftertouch) {
    return;
  }

  if(MIDI_CHANNEL && channel != MIDI_CHANNEL) return;

  //Aftertouch is only low-resolution.
  cc_msb[AT_SLOT] = pressure;
  cc_lsb[AT_SLOT] = 0;

}

void play_note(bool new_note) {

  //Send note V/oct
  uint16_t note_dac_value = cvtable[current_note-MIDI_NOTE_LOW];

  if(bend_merge && pitch_bend != 0) {
    int pb_amount;
    if(pitch_bend > 0) { //pitch bend up

      //Find the highest value we can pitch up to. Note that this will "compress" the pitch bend range if you are close to the end
      //of the range.
      uint16_t upper_note_dac = cvtable[min(current_note + PITCH_BEND_RANGE, MIDI_NOTE_HIGH)-MIDI_NOTE_LOW];

      //Interpolate position of pitch bend
      pb_amount = (long)pitch_bend * (upper_note_dac - note_dac_value) / 8192;


    } else { //pitch bend down
      uint16_t lower_note_dac = cvtable[max(current_note - PITCH_BEND_RANGE, MIDI_NOTE_LOW)-MIDI_NOTE_LOW];

      //Interpolate position of pitch bend
      pb_amount = (long)pitch_bend * (note_dac_value-lower_note_dac) / 8192;
    }

    note_dac_value += pb_amount;
  }

  #ifdef DEBUG_PRINT
  Serial.print("PLAYING NOTE ");
  Serial.print(current_note);
  Serial.print(" dac ");
  Serial.println(note_dac_value);
  #endif

  sendDac(NOTE_SLOT, note_dac_value);

  //Only do triggery things if it is a "new note" (i.e. not just pitch bend update)
  if(new_note) {

    //send trig pulse
    digitalWrite(TRIG_PIN, HIGH);
    trigger_on = true;
    trigger_on_time = micros();

    //Set gate on
    digitalWrite(GATE_PIN, HIGH);
    
    //Set velocity
    sendVelocity(note_velocity);
  }
}

void sendCC(byte slot) {
  
  //Make two 7-bit parts into one 12-bit value
  uint16_t value = map14to12(cc_msb[slot], cc_lsb[slot]);

  sendDac(slot, value);
}

void sendVelocity(uint8_t velocity) {
  //Scale 0-127 to 0-4095 by "reusing" three most significant bits
  sendDac(VELOCITY_SLOT, map14to12(velocity,velocity));
}


void sendDac(uint8_t slot, uint16_t value) {
  uint8_t pin = cs_pins[slot>>1]; //map slot to DAC chip (CS pin)
  uint8_t dac = slot % 2;         //map slot to DAC unit in chip

  uint16_t command = (dac?0x9000:0x1000); //DAC select + not shutdown
  if(cv_invert[slot]) value = 0x0FFF - value;
  if(!dac_gain[slot]) command |= 0x2000;  //Set "low gain" bit
  command |= value;

#ifdef DEBUG_PRINT
  if(slot>3) {
  Serial.print("Send ");
  Serial.print(value);
  Serial.print(" cs ");
  Serial.print(pin);
  Serial.print(" dac ");
  Serial.print(dac);
  Serial.print(" command ");
  Serial.println(command, HEX);
}
#endif

  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(pin, LOW);
  //Transfer command bytes separately
  SPI.transfer(command >> 8);
  SPI.transfer(command & 0x00FF);
  digitalWrite(pin, HIGH);
  SPI.endTransaction();
}

//Read switches and stuff into global variables for easy access
void readSwitches() {
  glide_merge = digitalRead(GLIDE_SWITCH);
  bend_merge = digitalRead(BEND_SWITCH);
  use_aftertouch = digitalRead(AFTERTOUCH_SWITCH);
  switchesRead = millis();
}