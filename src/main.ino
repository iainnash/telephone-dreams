#include <Audio.h>
#include <Bounce.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TimeLib.h>

// DEFINES
// Define pins used by Teensy Audio Shield
#define SDCARD_CS_PIN 10
#define SDCARD_MOSI_PIN 11
#define SDCARD_MISO_PIN 12
#define SDCARD_SCK_PIN 13
// And those used for inputs
#define HOOK_PIN 0
#define DIALPAD_PIN 1

// GLOBALS
// Audio initialisation code can be generated using the GUI interface at https://www.pjrc.com/teensy/gui/
// Inputs
AudioSynthWaveform waveform1;                       // To create the "beep" sfx
AudioInputI2S i2s2;                                 // I2S input from microphone on audio shield
AudioPlaySdWav playWav1;                            // Play 44.1kHz 16-bit PCM greeting WAV file
AudioRecordQueue queue1;                            // Creating an audio buffer in memory before saving to SD
AudioMixer4 mixer;                                  // Allows merging several inputs to same output
AudioOutputI2S i2s1;                                // I2S interface to Speaker/Line Out on Audio shield
AudioConnection patchCord1(waveform1, 0, mixer, 0); // wave to mixer
AudioConnection patchCord3(playWav1, 0, mixer, 1);  // wav file playback mixer
AudioConnection patchCord4(mixer, 0, i2s1, 0);      // mixer output to speaker (L)
AudioConnection patchCord6(mixer, 0, i2s1, 1);      // mixer output to speaker (R)
AudioConnection patchCord5(i2s2, 0, queue1, 0);     // mic input to queue (L)
AudioControlSGTL5000 sgtl5000_1;

// Filename to save audio recording on SD card
char filename[15];
// The file object itself
File frec;

// Use long 40ms debounce time on both switches
Bounce buttonHook = Bounce(HOOK_PIN, 300);
Bounce buttonDial = Bounce(DIALPAD_PIN, 20);

// Keep track of current state of the device
enum Mode
{
  Initialising,
  Ready,
  Prompting,
  Recording,
  Playing
};
Mode mode = Mode::Initialising;

float beep_volume = 0.08f; // not too loud :-)

// variables for writing to WAV file
unsigned long ChunkSize = 0L;
unsigned long Subchunk1Size = 16;
unsigned int AudioFormat = 1;
unsigned int numChannels = 1;
unsigned long sampleRate = 44100;
unsigned int bitsPerSample = 16;
unsigned long byteRate = sampleRate * numChannels * (bitsPerSample / 8); // samplerate x channels x (bitspersample / 8)
unsigned int blockAlign = numChannels * bitsPerSample / 8;
unsigned long Subchunk2Size = 0L;
unsigned long recByteSaved = 0L;
unsigned long NumSamples = 0L;
byte byte1, byte2, byte3, byte4;


void writeOutHeader() { // update WAV header with final filesize/datasize

//  NumSamples = (recByteSaved*8)/bitsPerSample/numChannels;
//  Subchunk2Size = NumSamples*numChannels*bitsPerSample/8; // number of samples x number of channels x number of bytes per sample
  Subchunk2Size = recByteSaved;
  ChunkSize = Subchunk2Size + 36;
  frec.seek(0);
  frec.write("RIFF");
  byte1 = ChunkSize & 0xff;
  byte2 = (ChunkSize >> 8) & 0xff;
  byte3 = (ChunkSize >> 16) & 0xff;
  byte4 = (ChunkSize >> 24) & 0xff;  
  frec.write(byte1);  frec.write(byte2);  frec.write(byte3);  frec.write(byte4);
  frec.write("WAVE");
  frec.write("fmt ");
  byte1 = Subchunk1Size & 0xff;
  byte2 = (Subchunk1Size >> 8) & 0xff;
  byte3 = (Subchunk1Size >> 16) & 0xff;
  byte4 = (Subchunk1Size >> 24) & 0xff;  
  frec.write(byte1);  frec.write(byte2);  frec.write(byte3);  frec.write(byte4);
  byte1 = AudioFormat & 0xff;
  byte2 = (AudioFormat >> 8) & 0xff;
  frec.write(byte1);  frec.write(byte2); 
  byte1 = numChannels & 0xff;
  byte2 = (numChannels >> 8) & 0xff;
  frec.write(byte1);  frec.write(byte2); 
  byte1 = sampleRate & 0xff;
  byte2 = (sampleRate >> 8) & 0xff;
  byte3 = (sampleRate >> 16) & 0xff;
  byte4 = (sampleRate >> 24) & 0xff;  
  frec.write(byte1);  frec.write(byte2);  frec.write(byte3);  frec.write(byte4);
  byte1 = byteRate & 0xff;
  byte2 = (byteRate >> 8) & 0xff;
  byte3 = (byteRate >> 16) & 0xff;
  byte4 = (byteRate >> 24) & 0xff;  
  frec.write(byte1);  frec.write(byte2);  frec.write(byte3);  frec.write(byte4);
  byte1 = blockAlign & 0xff;
  byte2 = (blockAlign >> 8) & 0xff;
  frec.write(byte1);  frec.write(byte2); 
  byte1 = bitsPerSample & 0xff;
  byte2 = (bitsPerSample >> 8) & 0xff;
  frec.write(byte1);  frec.write(byte2); 
  frec.write("data");
  byte1 = Subchunk2Size & 0xff;
  byte2 = (Subchunk2Size >> 8) & 0xff;
  byte3 = (Subchunk2Size >> 16) & 0xff;
  byte4 = (Subchunk2Size >> 24) & 0xff;  
  frec.write(byte1);  frec.write(byte2);  frec.write(byte3);  frec.write(byte4);
  frec.close();
  Serial.println("header written"); 
  Serial.print("Subchunk2: "); 
  Serial.println(Subchunk2Size); 
}

// Mode debugging
void print_mode(void)
{
  Serial.print("Mode switched to: ");
  // Initialising, Ready, Prompting, Recording, Playing
  if (mode == Mode::Ready)
    Serial.println(" Ready");
  else if (mode == Mode::Prompting)
    Serial.println(" Prompting");
  else if (mode == Mode::Recording)
    Serial.println(" Recording");
  else if (mode == Mode::Playing)
    Serial.println(" Playing");
  else if (mode == Mode::Initialising)
    Serial.println(" Initialising");
  else
    Serial.println(" Undefined");
}

// Non-blocking delay, which pauses execution of main program logic,
// but while still listening for input
void wait(unsigned int milliseconds)
{
  elapsedMillis msec = 0;

  while (msec <= milliseconds)
  {
    buttonHook.update();
    buttonDial.update();
    if (buttonHook.fallingEdge())
      Serial.println("Button (pin 0) Press");
    if (buttonDial.fallingEdge())
      Serial.println("Button (pin 1) Press");
    if (buttonHook.risingEdge())
      Serial.println("Button (pin 0) Release");
    if (buttonDial.risingEdge())
      Serial.println("Button (pin 1) Release");
  }
}

bool untilHook(bool status, unsigned int time)
{
  elapsedMillis msec = 0;
  while (msec <= time)
  {
    buttonHook.update();
    if (status && buttonHook.risingEdge())
    {
      return true;
    }
    if (!status && buttonHook.fallingEdge())
    {
      return true;
    }
  }
  return false;
}

int readDialpad(unsigned int time)
{
  unsigned int count = 0;
  elapsedMillis msec = 0;
  unsigned int startTime;
  bool reading = false;
  while (msec <= time || reading)
  {
    buttonDial.update();
    if (buttonDial.risingEdge())
    {
      startTime = msec;
      reading = true;
      count += 1;
    }
    if (reading && msec - startTime > 1100)
    {
      if (count == 10)
      {
        return 0;
      }
      return count;
    }
  }
  return -1;
}

// Script setup step
void setup()
{
  Serial.begin(9600);
  while (!Serial && millis() < 5000)
  {
    // wait for serial port to connect.
  }
  Serial.println("Serial set up correctly");
  print_mode();

  // record buffer audio memory
  AudioMemory(60);

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(HOOK_PIN, INPUT);
  pinMode(DIALPAD_PIN, INPUT);
  digitalWrite(HOOK_PIN, HIGH);
  digitalWrite(DIALPAD_PIN, HIGH);

  // Enable the audio shield, select input, and enable output
  sgtl5000_1.enable();
  // Define which input on the audio shield to use (AUDIO_INPUT_LINEIN / AUDIO_INPUT_MIC)
  sgtl5000_1.inputSelect(AUDIO_INPUT_MIC);
  // sgtl5000_1.adcHighPassFilterDisable(); //
  sgtl5000_1.volume(0.95);
  sgtl5000_1.micGain(20);

  mixer.gain(0, 1.0f);
  mixer.gain(1, 1.0f);

  // Play a beep to indicate system is online
  waveform1.begin(beep_volume, 440, WAVEFORM_SINE);
  wait(1000);
  waveform1.amplitude(0);
  delay(1000);

  pinMode(SDCARD_CS_PIN, OUTPUT);

  // Initialize the SD card
  SPI.setMOSI(SDCARD_MOSI_PIN);
  SPI.setSCK(SDCARD_SCK_PIN);
  if (!(SD.begin(SDCARD_CS_PIN)))
  {
    // stop here if no SD card, but print a message
    while (1)
    {
      Serial.println("Unable to access the SD card");
      delay(500);
      waveform1.begin(beep_volume, 840, WAVEFORM_SINE);
      wait(1000);
      waveform1.amplitude(0);
      delay(1000);
    }
  }
  else
  {
    Serial.println("SD card correctly initialized");
  }

  // Synchronise the Time object used in the program code with the RTC time provider.
  // See https://github.com/PaulStoffregen/Time
  // setSyncProvider(getTeensy3Time);

  // Define a callback that will assign the correct datetime for any file system operations
  // (i.e. saving a new audio recording onto the SD card)
  // FsDateTime::setCallback(dateTime);

  mode = Mode::Ready;
  print_mode();
}

void startRecording()
{
  // Find the first available file number
  //  for (uint8_t i=0; i<9999; i++) { // BUGFIX uint8_t overflows if it reaches 255
  for (uint16_t i = 0; i < 9999; i++)
  {
    // Format the counter as a five-digit number with leading zeroes, followed by file extension
    snprintf(filename, 11, " %05d.wav", i);
    // Create if does not exist, do not open existing, write, sync after write
    if (!SD.exists(filename))
    {
      break;
    }
  }
  frec = SD.open(filename, FILE_WRITE);
  Serial.println("Opened file !");
  if (frec)
  {
    Serial.print("Recording to ");
    Serial.println(filename);
    queue1.begin();
    mode = Mode::Recording;
    print_mode();
    recByteSaved = 0L;
  }
  else
  {
    Serial.println("Couldn't open file to record!");
  }
}

void continueRecording()
{
  // Check if there is data in the queue
  if (queue1.available() >= 2)
  {
    byte buffer[512];
    // Fetch 2 blocks from the audio library and copy
    // into a 512 byte buffer.  The Arduino SD library
    // is most efficient when full 512 byte sector size
    // writes are used.
    memcpy(buffer, queue1.readBuffer(), 256);
    queue1.freeBuffer();
    memcpy(buffer + 256, queue1.readBuffer(), 256);
    queue1.freeBuffer();
    // Write all 512 bytes to the SD card
    frec.write(buffer, 512);
    recByteSaved += 512;
  }
}

void stopRecording()
{
  // Stop adding any new data to the queue
  queue1.end();
  // Flush all existing remaining data from the queue
  while (queue1.available() > 0)
  {
    // Save to open file
    frec.write((byte *)queue1.readBuffer(), 256);
    queue1.freeBuffer();
    recByteSaved += 256;
  }
  writeOutHeader();
  // Close the file
  frec.close();
  Serial.println("Closed file");
  mode = Mode::Ready;
  print_mode();
}

void updateButtons()
{
  // First, read the buttons
  buttonHook.update();
  buttonDial.update();
}

int lastDial;

void loop()
{
  unsigned int startTime;

  switch (mode)
  {
  case Mode::Ready:
    updateButtons();
    if (buttonHook.risingEdge())
    {
      Serial.println("handset lifted :: prompting");
      mode = Mode::Prompting;
      print_mode();
    }
    else if (buttonDial.risingEdge())
    {
      // playLastRecording();
    }
    break;

  case Mode::Prompting:
    wait(2000);
    playWav1.play("00002.wav");
    while (!playWav1.isStopped())
    {
      if (untilHook(false, 800))
      {
        mode = Mode::Ready;
        break;
      }
    }

    waveform1.begin(beep_volume, 440, WAVEFORM_SINE);
    if (untilHook(false, 800))
    {
      waveform1.amplitude(0);
      mode = Mode::Ready;
      break;
    };
    waveform1.amplitude(0);
    if (untilHook(false, 800))
    {
      mode = Mode::Ready;
      break;
    };

    while (true)
    {
      if (untilHook(false, 100))
      {
        mode = Mode::Ready;
        break;
      }
      // dialpad
      int dial = readDialpad(5000);
      if (dial == 0)
      {
        mode = Mode::Recording;
        break;
      }
      else if (dial > 0)
      {
        lastDial = dial;
        mode = Mode::Playing;
        break;
      }
    }

    break;

  case Mode::Recording:
    // step 1 setup recording
    // step 2 record
    // Debug message
    Serial.println("Starting Recording");
    // Play the tone sound effect
    waveform1.begin(beep_volume, 440, WAVEFORM_SINE);
    wait(1250);
    waveform1.amplitude(0);
    // Start the recording function
    startRecording();

    startTime = millis();
    while (millis() - startTime <= 60000)
    {
      updateButtons();
      if (buttonHook.fallingEdge()) {
        break;
      }
      continueRecording();
    }
    stopRecording();

    waveform1.begin(beep_volume, 540, WAVEFORM_SINE);
    wait(1350);
    waveform1.amplitude(0);

    break;

  case Mode::Playing:
    // play a predefined dream
    delay(1000);
    playWav1.play("00000.wav");
    while (!playWav1.isStopped())
    {
      if (untilHook(false, 100))
      {
        mode = Mode::Prompting;
        break;
      }
    }
    mode = Mode::Prompting;
    break;

  case Mode::Initialising:

    break;
  }
}
