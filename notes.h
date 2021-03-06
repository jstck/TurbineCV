#ifndef __NOTES_H_
#define __NOTES_H_


//Size of buffer of notes. This needs to be a power of 2, less than 256. 32 is more than the number of fingers on four typical hands.
#define NOTE_BUFFER_SIZE 32

//Value representing no note
#define NO_NOTE -1

//Initialize me before using please
void note_init();

//Note on message, returns what note to play
int8_t note_on(int8_t note);

//Note off message, returns what note to play or negative if none
int8_t note_off(int8_t note);

//Get current note playing (negative if none)
int8_t get_current_note();

#endif