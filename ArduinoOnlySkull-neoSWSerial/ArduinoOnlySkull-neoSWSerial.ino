/***
 * Copyright 2018 Derwent Ready
 * Arduino-only version of the laughing skull halloween project.
 * Developed on Arduino Uno R3 but intended for Arduino Nano. Could probably be easily adapted to Teensy LC or similar.
 * Previous version used an Arduino to drive the servo and LEDs and a Raspberry Pi to play the laughing sound effect with limited success.
 * This was slightly cheaper than using an Arduino and MP3 board but I2C master/slave was a bit iffy and while using 
 * Since discovered the much more affordable DFPlayer board which can be bought for a lot less than other MP3 boards. It can play MP3, wav and WMA files.
 * Due to the size of the DFPlayer everything except speaker and battery should now fit inside the skull.
 * The project uses 2 LEDs, I used red but blue might also work for a more chilling effect.
 * Used pins:
 * D3             - PWM for right eye LED (could also use D4 if you don't want to fade or can work out a non-blocking bit banging version of the code)
 * D5             - PWM for left eye LED (could use D7 if not fading)
 * D9             - Digital servo pin
 * D10            - Software serial RX for DFPlayer
 * D11            - Software serial TX for DFPlayer
 * D12            - PIR input, triggers the laugh
 * Unused pins:
 * D1-2
 * D4
 * D6-8
 * D13            - Except internal LED on Arduino
 * A0-5           - One could be adapted to a volume adjustment pin.
 */
#include "Arduino.h"
//#include <SoftwareSerial.h>
#include <NeoSWSerial.h>
#include "DFRobotDFPlayerMini.h"
#include <Servo.h>

#define SRX   10
#define STX   11
#define SBAUD 9600
#define BAUD  115200 //for 

/*
 * The values to send to the servo to get it to open and close (angle of the motor)
 */
#define S_OPEN  10 // Nearly fully open. Could increase this value a bit to get more stability on the box.
#define S_CLOSE 135 //120 // Little way past midway point of a 180 degree servo, based on how the motor is mounted in the skull

#define LEDR   3
#define LEDL   5 // PWM pins for fade out, 3 and 5, digital only 4 and 7
#define SERVO  9 //4
#define PIR   12

#define DEFAULT_VOL 15 // Use 30 for non-amplified speaker

#define LAUGH_TRACK  4 // 1 - Change this to whichever track you want. Originally track 1 until I realised sound issues were due to the file being .wav. Converting to .mp3 solved these issues but haven't reorganised the SD card since.

#define MUSIC_DELAY 5000
#define MUSIC_TRACK 3 // DFPlayer counts files in the order they were saved to the microSD. For some reason it counts the audio tracks on the microSD from 1

bool PIRState = LOW;

bool playerAvailable = false;

bool idleMusic = false; // Set this to true to play music during idle times.
bool music = false; // Determines if music should be running right now

bool laughing = false;
bool jawDirection = false; // false = closing, true = opening
uint8_t jawOpens = 0;
uint8_t opensPerLaugh = 6;
long openDelay = 200; // Time in milliseconds for the jaw to remain open before closing, 120 too fast, 360 too slow. 240?
long lastJawTime; // The last time the jaw was moved in either direction

long lastLaughTime = 0;
long laughDelay = 1000; // Time in milliseconds before a laugh can trigger. 30 seconds to prevent early trigger when turned on and to prevent overly frequent laughs.

/**
 * Variables for LED fading after laugh
 */
bool fade;
uint8_t bright = 255;
long lastFadeTime = 0;
long fadeDelay = 15; // Time in milliseconds between 

//SoftwareSerial sSerial(SRX, STX);
NeoSWSerial sSerial(SRX, STX);
DFRobotDFPlayerMini player;
Servo servo;

void setup() {
  uint8_t maxAttempts = 6;
  uint8_t maxFlashes = 3;
  pinMode(LEDR, OUTPUT);
  pinMode(LEDL, OUTPUT);
  sSerial.begin(SBAUD);
  Serial.begin(BAUD);
  /* 
   *  DFPlayer can take a few seconds to initialise
   *  Instead of waiting for the maximum time before checking if it's initialised or 
   *  blocking indefinitely, we poll it a few times, up to the maximum time it should take to initialise.
   *  If it hasn't become available we'll continue on without it.
   */
  for(int i = 0; i < maxAttempts; i++) {
    if(!player.begin(sSerial)) {
      Serial.print("Error starting audio player on attempt ");
      Serial.print((i + 1));
      Serial.print(" of ");
      Serial.print(maxAttempts);
      Serial.println(" attempts.");
      Serial.println("Check connections and SD card");
      delay(1000);
    } else {
      playerAvailable = true;
      break; // We got it activated so quit early.
    }
  }
  if(playerAvailable) {
    Serial.println("Audio device available.");
    player.volume(30);
  } else {
    Serial.println("Couldn't initialise audio module. Continuing without it. Consider restarting for full effect. If you keep seeing this message check wiring and SD card.");
    maxFlashes -= 1; // Disable one flash to indicate the audio module failed to load.
  }
  servo.attach(SERVO);
  servo.write(S_CLOSE);
  Serial.println("Initialised");
  /**
   * Flash the eyes to indicate ready state if not attached to serial monitor.
   */
  digitalWrite(LEDR, HIGH);
  delay(250);
  digitalWrite(LEDL, HIGH);
  delay(250);
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDL, LOW);
  for(int i = 0; i <= maxFlashes; i++) {
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDL, HIGH);
    delay(450);
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDL, LOW);
    delay(250);
  }
  Serial.println("Ready!");
}

void loop() {
  bool state = digitalRead(PIR);
  if(state == HIGH && PIRState == LOW) { // Prevents the animation retriggering every loop once the PIR triggers. The PIR will stay high for several loops.
    if(millis() - lastLaughTime < laughDelay) { // Make sure the PIR hasn't triggered again before the 
      Serial.println("Too soon to trigger!");
    } else {
      Serial.println("INTRUDER! DEPLOY SPOOKUMS!");
      if(playerAvailable) {
        player.play(LAUGH_TRACK); // Play the evil laugh sound effect.
      }
      // Turn the LEDs full on for shock value.
      digitalWrite(LEDR, HIGH);
      digitalWrite(LEDL, HIGH);
      laughing = true;
      music = false;
      fade = false; // Reset LEDs to not fading to prevent early activations making the eyes look wrong.
      bright = 255;
      lastLaughTime = millis();
    }
  }
  // This if isn't really needed any more but good for debugging when trying to set PIR timeout.
  if(state == LOW && PIRState == HIGH) {
    Serial.println("Intruder's gone.");
  }
  // Laugh has been triggered, let's start the laugh animation on the servo.
  if(laughing) {
    if(!servo.attached()) {      
      servo.attach(SERVO);
    }
    // Open and close the jaw several times before setting the 
    if(jawOpens < opensPerLaugh) {
      if(millis() - openDelay >= lastJawTime) {
        if(jawDirection == true) {
          Serial.println("Opening jaw");
          servo.write(S_OPEN);
          jawDirection = false;
        } else {
          Serial.println("Closing jaw");
          servo.write(S_CLOSE);
          jawDirection = true;
          jawOpens++; // Put this here so we get one full open and close for each open, won't get a final close if put in the jawDirection == true block
        }
        lastJawTime = millis();
      }
    }
    if(jawOpens == opensPerLaugh) {
      // Done laughing so reset the jaw
      if(millis() - openDelay >= lastJawTime) {
        laughing = false;
        jawDirection = true;
        jawOpens = 0;
        servo.detach();
        lastJawTime = millis();
        fade = true;
      }
    }
  }
  if(fade) {
    if(bright > 0 && millis() - lastFadeTime >= fadeDelay) {
      bright -= 1;
      lastFadeTime = millis();
    }
    analogWrite(LEDR, bright);
    analogWrite(LEDL, bright);
    if(bright == 0) {
      bright = 255;
      fade = false;
    }
  }
  PIRState = state; // Set the current PIR state for future loops to check against.
  if(idleMusic) {
    if(millis() - lastLaughTime >= MUSIC_DELAY) {
      if(!music) {
        Serial.println("Play some spooky music!");
        if(playerAvailable) {
          player.play(MUSIC_TRACK);
        }
        music = true;
      }
    }
    if(music && playerAvailable) {
      if(player.readType() == DFPlayerPlayFinished && player.read() == MUSIC_TRACK) {
        // Restart
        Serial.println("Starting the music over!");
        music = false;
      }
    }
  }
}
