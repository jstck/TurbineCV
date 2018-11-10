#include "notes.h"


//Buffer to keep track of which notes where played in what order

int8_t note_buffer[NOTE_BUFFER_SIZE];

uint8_t buffer_head;
uint8_t buffer_tail;

//Bit field for all MIDI notes that could possibly be played (goes from 0-127, 16 bytes of stuff)
uint8_t notes_on[16];

//Macros to increment and decrement buffer positions with wraparound
#define p_inc(x) x=(x+1)%NOTE_BUFFER_SIZE
#define p_dec(x) x=(x+NOTE_BUFFER_SIZE-1)%NOTE_BUFFER_SIZE;


//Functions for 127-note wide bit field
void set_note(uint8_t note)
{
  uint8_t pos_byte = note >> 3;
  uint8_t pos_bit = note & 0x07; //Last 3 bits

  uint8_t bitmask = 1 << pos_bit;

  notes_on[pos_byte] |= bitmask;
}

void clear_note(uint8_t note)
{
  uint8_t pos_byte = note >> 3;
  uint8_t pos_bit = note & 0x07; //Last 3 bits

  uint8_t bitmask = 1 << pos_bit;

  notes_on[pos_byte] &= ~bitmask;
}

bool get_note_state(uint8_t note)
{
  uint8_t pos_byte = note >> 3;
  uint8_t pos_bit = note & 0x07; //Last 3 bits

  uint8_t bitmask = 1 << pos_bit;

  return notes_on[pos_byte] & bitmask;
}


//Initialize me please
void note_init() {

	for(uint8_t	i=0; i<7; ++i) {
		notes_on[i] = 0;
	}

	for(uint8_t	i=0; i<NOTE_BUFFER_SIZE; ++i) {
		note_buffer[i] = NO_NOTE;
	}

	buffer_head = 0;
	buffer_tail = NOTE_BUFFER_SIZE-1;
}

//Note on message, returns what note to play
int8_t note_on(int8_t note) {

	set_note(note);

	if(note_buffer[buffer_head] == NO_NOTE || note_buffer[buffer_head] == note) {
		note_buffer[buffer_head] = note;
	} else {
		p_inc(buffer_head);
		note_buffer[buffer_head] = note;
	}
	
	//If we've gone a whole lap around, move the tail and just forget the last note
	if (buffer_head == buffer_tail) {
		p_inc(buffer_tail);
	}

	return note;
}

//Note off message, returns what note to play or negative if none
int8_t note_off(int8_t note) {
	clear_note(note);

	//Check if it's the currently playing note that is turning off
	if(note_buffer[buffer_head] == note) {
		note_buffer[buffer_head] = NO_NOTE;
		p_dec(buffer_head);

		//Backtrack until we find a note still playing, or get to an empty buffer
		while(true) {

			//Nothing more to say
			if(buffer_head == buffer_tail) {
				//Reset things to start state just because
				note_init();

				return NO_NOTE;
			}

			int8_t	top_note = note_buffer[buffer_head];

			//This is a currently playing note, let's stay here
			if(get_note_state(top_note)) {
				return top_note;
			}

			//This note has been released earlier. Pop it off, go back further.
			note_buffer[buffer_head] = NO_NOTE;
			p_dec(buffer_head);
		}
	}
	//Still playing the same note
	else {
		return note_buffer[buffer_head];
	}
}

//Get current note playing (negative if none)
int8_t get_current_note() {
	return note_buffer[buffer_head];
}




void dump_buffer() {
	int head_spaces = 2+buffer_head*4;
	int tail_spaces = 2+buffer_tail*4;

	for(int i=0; i<head_spaces; i++) Serial.print(" ");
	Serial.println("v");
	for(int i=0; i<NOTE_BUFFER_SIZE; i++) {
		Serial.print(" "); Serial.print(note_buffer[i]);
		if(i<NOTE_BUFFER_SIZE-1) Serial.print("|");
	}
	Serial.println("");
	for(int i=0; i<tail_spaces; i++) Serial.print(" ");
	Serial.println("^");


	for(int i=1; i<=127; i++) {
		if(get_note_state(i)) {
			Serial.print(i);
			Serial.print(" ");
		}
	}
	Serial.println("");
	Serial.println("");

}
