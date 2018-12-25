#include <MIDI.h>
#include <SPI.h>
#include <FixedPoints.h>
#include <FixedPointsCommon.h>

#include "notes.h"
#include "cvtable.h"

//Define hardware type for pin assignments
#define EMI_TEENSY_LC
#include "hardware.h"

//The MIDI channel to listen on. Set to 0 for "all channels"
#define MIDI_CHANNEL 0

//Pitch bend range, in number of half tones (12 = one octave)
#define PITCH_BEND_RANGE 5

//Uncomment this to get debug output on serial console. May add some latency to MIDI output. Only do this on
//arduinos where serial console does not use the same hardware serial port as MIDI
//#define DEBUG_PRINT

#define UNUSED_CC 255
#define HIRES_CC_AUTO 254

//Number of DAC chips (SPI devices) and corresponding DAC channels (2 per chip for MCP4822)
#define NUM_DACS 3
#define DAC_CHANNELS 6



//MIDI CC messages used for the different outputs
uint8_t cc_list[DAC_CHANNELS] = {
  UNUSED_CC,  //Note pitch
  5,          //Portamento time (glide)
  UNUSED_CC,  //Pitch bend
  2,          //Breath
  1,          //Mod wheel
  UNUSED_CC   //Velocity
};

//"DAC slots" where various specially treated things are. DAC 0 is slots 0 and 1, DAC 1 is 2 and 3, DAC 2 is 4 and 5.
#define NOTE_SLOT 0
#define PORTAMENTO_SLOT 1
#define BEND_SLOT 2
#define AT_SLOT 3 //Aftertouch goes in breath slot
#define VELOCITY_SLOT 5



//High-resolution CC messages. Typically "regular CC" + 32. Can be manually set here.
//If set to HIRES_CC_AUTO, these are automatically assigned to CC+32 (where main CC is in 1-31 range).
//If hi-res is unwanted, set this to UNUSED_CC to block it off
//Slots that are not used for CC's are disabled
uint8_t cc_hires[DAC_CHANNELS] = {
  UNUSED_CC,
  HIRES_CC_AUTO,
  UNUSED_CC,
  HIRES_CC_AUTO,
  HIRES_CC_AUTO,
  UNUSED_CC,
};


//Input pin for portamento amount. Comment out if unused
//#define PORTA_SCALER_PIN A0

//How long to run the trigger pulse for (very roughly) in microseconds
#define TRIGGER_WIDTH 200

//How long to turn the activity LED on for, in milliseconds
#define MIDI_ACT_TIME 50

//How often to read onboard switches, in milliseconds
#define SWITCH_READ_INTERVAL 500

//Chip select pins for SPI bus to DACs. The board default pins are used for MOSI and SCK.
uint8_t cs_pins[DAC_CHANNELS/2] = {CS_DAC1, CS_DAC2, CS_DAC3};

//Flags to set high=1 (0-4V) or low=0 gain (0-2V) per DAC.
uint8_t dac_gain[DAC_CHANNELS] = {1, 0, 0, 0, 0, 0};

//Flags to invert DAC channel. Normal operation (0) is lowest voltage at 0 CC value / low note, 1 inverts this.
uint8_t cv_invert[DAC_CHANNELS] = {0, 0, 0, 0, 0, 0};


//Map two 7-bit parts into one 12-bit value
#define map14to12(msb, lsb)  ((((uint16_t)msb << 5) | (lsb >> 2)) & 0X0FFF)


bool trigger_on = false;
int8_t current_note = NO_NOTE; //MIDI value of currently playing note
int8_t last_note = NO_NOTE; //MIDI value of whatever note was playing last, even if it's no longer active
uint8_t note_velocity = 0; //Velocity of currently playing note
unsigned long trigger_on_time;
int pitch_bend = 0; //Signed value, -8192 - +8191
int pitch_bend_offset = 0; //Amount of offset on note DAC value from pitch bend

bool glide_merge;
bool bend_merge;
bool use_aftertouch;
unsigned long switchesRead;
unsigned long lastActivity;
bool ledActive = false;

//Portamento/glide settings and state

//Portamento update rate in milliseconds
#define PORTA_INTERVAL 5

//Constant factor for proportional amount. Decrease this if interval increases
SQ15x16 porta_prop_k = 400.0;

int porta_scaler = 512; //Init to half value, used if pin is not defined
int8_t porta_direction; //Portamento direction (+1 for up, -1 for down)
int8_t porta_cc = 0;

SQ15x16 porta_cc_factor = 0.0; //Factor based on CC value (CC^1.5)
SQ15x16 porta_scaler_factor = 22.6; //Factor based on scaler port (sqrt(analog value)), init to approx sqrt(512)

SQ15x16 porta_linear_amount = 0.02; //Linear glide amount per tick

SQ15x16 porta_current;
int porta_target;


bool porta_active = false; //PB glide + cc both active
bool in_portamento = false; //Portamento is happening right now

int current_note_dac;

unsigned long last_porta_update;


//Do midi with default settings.
//MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);
MIDI_CREATE_DEFAULT_INSTANCE();

//High and low bytes of each CC.
uint8_t cc_msb[DAC_CHANNELS];
uint8_t cc_lsb[DAC_CHANNELS];


//Keep track of wether or not we have seen a hi-res message for this CC.
bool hires_seen[DAC_CHANNELS] = {false, false, false, false, false, false};

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
  for(uint8_t i=0; i<DAC_CHANNELS/2; ++i) {
    pinMode(cs_pins[i], OUTPUT);
    digitalWrite(cs_pins[i], HIGH);
  }
  for(uint8_t i=0; i<DAC_CHANNELS; ++i) {
    cc_msb[i] = 0;
    cc_lsb[i] = 0;
    if(cc_list[i] != UNUSED_CC) {
      sendCC(i);
    }
  }

  //Setup high-resolution CC
  for(uint8_t i=0; i<DAC_CHANNELS; ++i) {
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

  //Signal midi activity led to indicate things are ready
  midiActivity();

#ifdef DEBUG_PRINT
  Serial.println("TurbineCV ready");
#endif
}


void loop() {

  //Poll for MIDI data. This will trigger handler functions if anything comes in.
  MIDI.read();

  //Handle various timed tasks, in order of urgency.
  //As soon as we "hit one" exit to repeat loop so each run does not take too much time.

  //Microsecond-level events first
  unsigned long now = micros();

  //See if it's time to turn trigger off
  if(trigger_on && now >= trigger_on_time + TRIGGER_WIDTH) {
    digitalWrite(TRIG_PIN, LOW);
    trigger_on = false;
    return;
  }

  //Then millisecond-level ones
  now = millis();

  if(in_portamento && now >= last_porta_update + PORTA_INTERVAL) {
    updatePortamento();
    return;
  }

  //These are nice if they're donein a timely fashion, but can really wait.
  if(ledActive && now >= lastActivity + MIDI_ACT_TIME) {
    digitalWrite(LED_PIN, LOW);
    ledActive = false;
    return;
  }

  if(now >= switchesRead + SWITCH_READ_INTERVAL) {
    readSwitches();
    return;
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

  midiActivity();

  for(uint8_t i=0; i<DAC_CHANNELS; ++i) {
    if(number==cc_list[i]) {

      //Ignore this value if it is a value to be overridden by aftertouch
      if(use_aftertouch && i == AT_SLOT) return;

      //Check if this is a glide/portamento update.
      //The dedicated portamento CV output is dealt with as a regular hi-res CC, but the glide merge function
      //only looks at 7-bit CC for simplicity.
      if(i == PORTAMENTO_SLOT && glide_merge) {
        porta_active = (value > 0); //THIS IS PORTAAAA!
        if(value != porta_cc) {
          porta_cc = value;
          update_porta_proportional();
        }
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

    if(number==cc_hires[i]) {
      //Read LSB ("high-res" part), and send it right away.
      cc_lsb[i] = value;
      hires_seen[i] = true;

#ifdef DEBUG_PRINT_CC
      Serial.print("Sending slot ");
      Serial.print(i);
      Serial.println(" (high-res)");
#endif
      sendCC(i);
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

  midiActivity();

  if(bend == pitch_bend) return; //Don't do anything if value is repeated

  pitch_bend = bend;
  int pb_dac = (bend+8192) / 4; //Scale pb value from -8192 - 8191 down to "dac range", 0-4095
  sendDac(BEND_SLOT, pb_dac);
  

  if(bend_merge) {
    updateBendOffset();
    play_note(false);  
  }
}

void updateBendOffset() {
  if(pitch_bend == 0) {
    pitch_bend_offset = 0;
  } else if(pitch_bend > 0) { //pitch bend up

    //Find the highest value we can pitch up to. Note that this will "compress" the pitch bend range if you are close to the end
    //of the range.
    uint16_t upper_note_dac = cvtable[min(last_note + PITCH_BEND_RANGE, MIDI_NOTE_HIGH)-MIDI_NOTE_LOW];

    //Interpolate position of pitch bend
    pitch_bend_offset = (long)pitch_bend * (upper_note_dac - current_note_dac) / 8192;


  } else { //pitch bend down
    uint16_t lower_note_dac = cvtable[max(last_note - PITCH_BEND_RANGE, MIDI_NOTE_LOW)-MIDI_NOTE_LOW];

    //Interpolate position of pitch bend
    pitch_bend_offset = (long)pitch_bend * (current_note_dac-lower_note_dac) / 8192;
  }
}


void midiNoteOn(byte channel, byte note, byte velocity) {

  #ifdef DEBUG_PRINT
  //Serial.print("Chan: ");
  //Serial.print(channel);
  Serial.print("Note on: ");
  Serial.println(note);
  //Serial.print(" vel: ");
  //Serial.println(velocity);
  #endif

  if(MIDI_CHANNEL && channel != MIDI_CHANNEL) return;

  midiActivity();

  //Just ignore notes outside of range (these could be "folded back in", but we just ignore them)
  if(note < MIDI_NOTE_LOW || note > MIDI_NOTE_HIGH) return;

  //Is portamento happening as a result of this?
  if(current_note != NO_NOTE && porta_active) {
    in_portamento = true;
  }

  current_note = note_on(note);
  if(current_note != NO_NOTE) last_note = current_note;
  
  note_velocity = velocity;

  play_note(true);

}


void midiNoteOff(byte channel, byte note, byte velocity) {

  #ifdef DEBUG_PRINT
  //Serial.print("Chan: ");
  //Serial.print(channel);
  Serial.print("Note off: ");
  Serial.println(note);
  //Serial.print(" vel: ");
  //Serial.println(velocity);
  #endif

  if(MIDI_CHANNEL && channel != MIDI_CHANNEL) return;

  midiActivity();

  if(note < MIDI_NOTE_LOW || note > MIDI_NOTE_HIGH) return;

  int8_t new_note = note_off(note);

  //Don't do anything if note didn't change
  if(new_note == current_note) {
    return;
  }

  //If no note is playing, turn off gate and stop portamento
  if(new_note == NO_NOTE) {
    digitalWrite(GATE_PIN, LOW);
    current_note = NO_NOTE;
    in_portamento = false;

  } else {
    current_note = new_note;
    last_note = current_note;
    play_note(true);
  }

  
}

void midiAftertouchChannel(byte channel, byte pressure) {
  
  //Bail out if not set to use aftertouch (via switch)
  if(!use_aftertouch) {
    return;
  }

  if(MIDI_CHANNEL && channel != MIDI_CHANNEL) return;

  midiActivity();

  #ifdef DEBUG_PRINT
  Serial.print("Chan: ");
  Serial.print(channel);
  Serial.print(" aftertouch: ");
  Serial.println(pressure);
  #endif

  //Aftertouch is only low-resolution.
  cc_msb[AT_SLOT] = pressure;
  cc_lsb[AT_SLOT] = pressure;

  sendCC(AT_SLOT);

}

void play_note(bool new_note) {

  //Exit on "invalid notes" that can't be played anyway
  if(current_note < MIDI_NOTE_LOW || current_note > MIDI_NOTE_HIGH) {
    return;
  }

  //Send note V/oct
  uint16_t note_dac_value = cvtable[current_note-MIDI_NOTE_LOW];

  if(in_portamento) {
    //???
  }

  //Recalculate amount of pitch bend offset if changing note
  if(new_note) updateBendOffset();

  note_dac_value += pitch_bend_offset;

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

#ifdef KEIN_DEBUG_PRINT
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

//Read switches and stuff into global variables for easy access
void readSwitches() {
  glide_merge = digitalRead(GLIDE_SWITCH);
  bend_merge = digitalRead(BEND_SWITCH);
  use_aftertouch = digitalRead(AFTERTOUCH_SWITCH);

  //If porta scaler pot changes value, do some of the math here
#ifdef PORTA_SCALER_PIN
  int ps_new = analogRead(PORTA_SCALER_PIN);
  if(ps_new != porta_scaler) {
    porta_scaler = ps_new;
    porta_scaler_factor = sqrt(porta_scaler);
    update_porta_proportional();
  }

  porta_scaler_factor = sqrt(porta_scaler);
#endif

  switchesRead = millis();
}



/*
  Portamento math:
  A simple exponential curve is done by periodically updating the note dac value

  new value = current value + A * (target value - current value) + B * direction

  this step (new value - current value) is basically the differential in an ordinary first-order differential
  equation. 

  The linear term (B*direction, where direction is +/- 1) is there to make sure we reach the target value in
  reasonable (non-infinite) time.

*/

//The proportional factor of the portamento calculation only changes when cc or scaler pot changes value and is only
//recalculated when needed
void update_porta_proportional() {
  porta_proportional = porta_prop_k / (porta_cc_factor * porta_scaler_factor);
}

//Calculate portamento step
float porta_step() {
  return porta_current + (porta_target - porta_current) * porta_proportional + porta_linear_amount * porta_direction;
}

void updatePortamento() {

  SQ15x16 step = porta_step();

  SQ15x16 new_porta_dac = porta_current + step;
  int new_dac = roundFixed(new_porta_dac);

  //Check for overshoot or having reached target
  if( (porta_direction > 0 && new_dac >= porta_target) || (porta_direction < 0 && new_dac <= porta_target) ) {
    new_dac = porta_target;
    in_portamento = false;
  }

  if(new_dac != current_note_dac) {
    sendDac(NOTE_SLOT, new_dac);
    current_note_dac = new_dac;
  }

  last_porta_update = millis();
}

void midiActivity() {
  lastActivity = millis();
  digitalWrite(LED_PIN, HIGH);
  ledActive = true;
}