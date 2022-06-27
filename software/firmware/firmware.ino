#include <usbmidi.h>
#include <midi_serialization.h>
#include <TimerOne.h>

//Inspired by floppotron and Moppy 2.0
//For Leonardo

#define FLOPPY_MIN_FREQ 20.0
#define FLOPPY_MAX_FREQ 450.0
#define BUZZER_MIN_FREQ 100.0
#define BUZZER_MAX_FREQ 1047.0 //above 520 Hz frequency is non-exact, above 1046 incorrect
#define TRANSF_MIN_FREQ 20.0
#define TRANSF_MAX_FREQ 270.0
#define HDD_MAX_PULSE 45000ul
#define NUM_STEPS 80
#define CHANNELS_COUNT 8
#define PWM_DUTY 0.95
#define TIMER_PERIOD 32

enum channel_type {
  CHANNEL_TYPE_STD_FLOPPY, //A-ENABLE, B-DIR, C-STEP
  CHANNEL_TYPE_CUSTOM_DRV_FLOPPY, //A-ENABLE, B-COILA C-COILB
  CHANNEL_TYPE_BUZZER, //A-output
  CHANNEL_TYPE_HDD, //A-coil
  CHANNEL_TYPE_TRANSF, //A-coil
};

struct channel_config {
  const channel_type c_t;
  const int pinA;
  const int pinB;
  const int pinC;
  const float minFr;
  const float maxFr;
};
struct channel_state {
  u8 varA;
  u8 phase;
  u8 steps;
  bool dir;
  unsigned int del;
  unsigned int delA;
  unsigned int delB;
  unsigned int cycles;
  unsigned long t_shutdown;
  u8 playingNote;
  bool remapped;  
};

static const int CUSTOM_DRV_SEQUENCE[4][2] = {
  {0, 0}, 
  {0, 1}, 
  {1, 1}, 
  {1, 0}
};

static const channel_config channel_cfgs[CHANNELS_COUNT] = {
  {CHANNEL_TYPE_CUSTOM_DRV_FLOPPY, 0, 1, 2,    FLOPPY_MIN_FREQ, FLOPPY_MAX_FREQ},
  {CHANNEL_TYPE_STD_FLOPPY,        3, 4, 5,    FLOPPY_MIN_FREQ, FLOPPY_MAX_FREQ},
  {CHANNEL_TYPE_STD_FLOPPY,        6, 7, 8,    FLOPPY_MIN_FREQ, FLOPPY_MAX_FREQ},
  {CHANNEL_TYPE_STD_FLOPPY,        9, 10, 11,  FLOPPY_MIN_FREQ, FLOPPY_MAX_FREQ},
  {CHANNEL_TYPE_TRANSF,            12, -1, -1, TRANSF_MIN_FREQ, TRANSF_MAX_FREQ},
  {CHANNEL_TYPE_BUZZER,            A0, -1, -1, BUZZER_MIN_FREQ, BUZZER_MAX_FREQ},
  {CHANNEL_TYPE_HDD,               A1, -1, -1, 0, 10000},
  {CHANNEL_TYPE_HDD,               A2, -1, -1, 0, 10000},
};
//CHANNELS ARE HARDCODED TO INCREASE PERFORMANCE!!! CHANGE MAIN LOOP WHEN REQUIRED

channel_state channel_states[CHANNELS_COUNT];

MidiToUsb mtou;
unsigned long da = 0;
unsigned long db = 0;
bool phaseCorrupt = false;

unsigned int freqToDelayTimerPer(float freq) {
  return (1000000ull / freq) / TIMER_PERIOD;
}

void invertDir(int ch) {
  switch(channel_cfgs[ch].c_t) {
    case CHANNEL_TYPE_STD_FLOPPY:
      channel_states[ch].dir = !channel_states[ch].dir;
      digitalWrite(channel_cfgs[ch].pinB, channel_states[ch].dir);
      break;
    case CHANNEL_TYPE_CUSTOM_DRV_FLOPPY:
      channel_states[ch].dir = !channel_states[ch].dir;
      break;
  }
}

void syncStep(int ch) {
  switch(channel_cfgs[ch].c_t) {
    case CHANNEL_TYPE_STD_FLOPPY:
      digitalWrite(channel_cfgs[ch].pinC, true);
      delayMicroseconds(channel_states[ch].del*TIMER_PERIOD/2);
      digitalWrite(channel_cfgs[ch].pinC, false);
      delayMicroseconds(channel_states[ch].del*TIMER_PERIOD/2);
      channel_states[ch].steps++;
      if(channel_states[ch].steps > NUM_STEPS) {
        invertDir(ch);
        channel_states[ch].steps = 0;
      }
      break;
    case CHANNEL_TYPE_CUSTOM_DRV_FLOPPY:
      digitalWrite(channel_cfgs[ch].pinA, true);
      for(int i = 0; i < 2; i++) {
        digitalWrite(channel_cfgs[ch].pinB, CUSTOM_DRV_SEQUENCE[channel_states[ch].varA][0]);
        digitalWrite(channel_cfgs[ch].pinC, channel_states[ch].dir ^ CUSTOM_DRV_SEQUENCE[channel_states[ch].varA][1]);
        channel_states[ch].varA = (channel_states[ch].varA + 1) % 4;
        delayMicroseconds(channel_states[ch].del*TIMER_PERIOD * (PWM_DUTY/2));
      }
      digitalWrite(channel_cfgs[ch].pinA, false);
      channel_states[ch].steps++;
      if(channel_states[ch].steps > NUM_STEPS) {
        invertDir(ch);
        channel_states[ch].steps = 0;
      }
      delayMicroseconds(channel_states[ch].del*TIMER_PERIOD * (1-PWM_DUTY));
      break;
    case CHANNEL_TYPE_BUZZER:
      //varA - velocity(0-255)(PWM duty)
      digitalWrite(channel_cfgs[ch].pinA, true);
      delayMicroseconds(channel_states[ch].del*TIMER_PERIOD*channel_states[ch].varA/256);
      digitalWrite(channel_cfgs[ch].pinA, false);
      delayMicroseconds(channel_states[ch].del*TIMER_PERIOD*(256-channel_states[ch].varA)/256);
      break;
    case CHANNEL_TYPE_HDD:
      //del - velocity(pulse length in timer periods)
      if(channel_states[ch].del != 0) {
        digitalWrite(channel_cfgs[ch].pinA, true);
        delayMicroseconds(channel_states[ch].del*TIMER_PERIOD);
        channel_states[ch].del = 0;
        digitalWrite(channel_cfgs[ch].pinA, false);
      }
      break;
    case CHANNEL_TYPE_TRANSF:
      //varA - velocity(0-255)(PWM duty)
      digitalWrite(channel_cfgs[ch].pinA, true);
      delayMicroseconds(channel_states[ch].del*TIMER_PERIOD*channel_states[ch].varA/128);
      digitalWrite(channel_cfgs[ch].pinA, false);
      delayMicroseconds(channel_states[ch].del*TIMER_PERIOD*(128-channel_states[ch].varA)/128);
      break;
  }
}

#pragma GCC push_options
#pragma GCC optimize("Ofast")
template<int ch>
void asyncStep_stdfloppy() {
  if(channel_states[ch].phase == 0 && channel_states[ch].cycles > channel_states[ch].delA) {
    digitalWrite(channel_cfgs[ch].pinC, 1);
    channel_states[ch].phase = 1;
    channel_states[ch].cycles = 0;
  } else if(channel_states[ch].phase == 1 && channel_states[ch].cycles > channel_states[ch].delB) {
    digitalWrite(channel_cfgs[ch].pinC, 0);
    channel_states[ch].steps++;
    if(channel_states[ch].steps > NUM_STEPS) {
      invertDir(ch);
      channel_states[ch].steps = 0;
    }
    channel_states[ch].phase = 0;
    channel_states[ch].cycles = 0;
  }
}

template<int ch>
void asyncStep_customdrvfloppy() {
  if(channel_states[ch].phase == 0 && channel_states[ch].cycles > channel_states[ch].delA) {
    digitalWrite(channel_cfgs[ch].pinA, true);
    digitalWrite(channel_cfgs[ch].pinB, CUSTOM_DRV_SEQUENCE[channel_states[ch].varA][0]);
    digitalWrite(channel_cfgs[ch].pinC, channel_states[ch].dir ^ CUSTOM_DRV_SEQUENCE[channel_states[ch].varA][1]);
    channel_states[ch].varA = (channel_states[ch].varA + 1) % 4;
    channel_states[ch].phase = 1;
    channel_states[ch].cycles = 0;
  } else if(channel_states[ch].phase == 1 && channel_states[ch].cycles > channel_states[ch].delB) {
    digitalWrite(channel_cfgs[ch].pinB, CUSTOM_DRV_SEQUENCE[channel_states[ch].varA][0]);
    digitalWrite(channel_cfgs[ch].pinC, channel_states[ch].dir ^ CUSTOM_DRV_SEQUENCE[channel_states[ch].varA][1]);
    channel_states[ch].varA = (channel_states[ch].varA + 1) % 4;
    channel_states[ch].phase = 2;
    channel_states[ch].cycles = 0;
  } else if(channel_states[ch].phase == 2 && channel_states[ch].cycles > channel_states[ch].delB) {
    digitalWrite(channel_cfgs[ch].pinA, false);
    channel_states[ch].steps++;
    if(channel_states[ch].steps > NUM_STEPS) {
      invertDir(ch);
      channel_states[ch].steps = 0;
    }
    channel_states[ch].phase = 0;
    channel_states[ch].cycles = 0;
  }
}

template<int ch>
void asyncStep_buzzer() {
  if(channel_states[ch].phase == 0 && channel_states[ch].cycles > channel_states[ch].delA) {
    digitalWrite(channel_cfgs[ch].pinA, true);
    channel_states[ch].phase = 1;
    channel_states[ch].cycles = 0;
  } else if(channel_states[ch].phase == 1 && channel_states[ch].cycles > channel_states[ch].delB) {
    digitalWrite(channel_cfgs[ch].pinA, false);
    channel_states[ch].phase = 0;
    channel_states[ch].cycles = 0;
  }
}

template<int ch>
void asyncStep_hdd() {
  if(channel_states[ch].phase == 0) {
    digitalWrite(channel_cfgs[ch].pinA, true);
    channel_states[ch].phase = 1;
    channel_states[ch].cycles = 0;
  } else if(channel_states[ch].phase == 1 && channel_states[ch].cycles > channel_states[ch].del) {
    digitalWrite(channel_cfgs[ch].pinA, false);
    channel_states[ch].del = 0;
    channel_states[ch].phase = 0;
    channel_states[ch].cycles = 0;
  }
}

template<int ch>
void asyncStep_transf() {
  if(channel_states[ch].phase == 0 && channel_states[ch].cycles > channel_states[ch].delA) {
    digitalWrite(channel_cfgs[ch].pinA, true);
    channel_states[ch].phase = 1;
    channel_states[ch].cycles = 0;
  } else if(channel_states[ch].phase == 1 && channel_states[ch].cycles > channel_states[ch].delB) {
    digitalWrite(channel_cfgs[ch].pinA, false);
    channel_states[ch].phase = 0;
    channel_states[ch].cycles = 0;
  }
}

void mainchannel_statesLoop() {
  //ugly piece of code...
  if(phaseCorrupt) {
    if(channel_states[0].del == 0 && channel_states[0].phase != 0) {
      channel_states[0].cycles++;
      asyncStep_customdrvfloppy<0>();
    } else if(channel_states[1].del == 0 && channel_states[1].phase != 0) {
      channel_states[1].cycles++;
      asyncStep_stdfloppy<1>();
    } else if(channel_states[2].del == 0 && channel_states[2].phase != 0) {
      channel_states[2].cycles++;
      asyncStep_stdfloppy<2>();
    } else if(channel_states[3].del == 0 && channel_states[3].phase != 0) {
      channel_states[3].cycles++;
      asyncStep_stdfloppy<3>();
    } else if(channel_states[4].del == 0 && channel_states[4].phase != 0) {
      channel_states[4].cycles++;
      asyncStep_transf<4>();
    } else if(channel_states[5].del == 0 && channel_states[5].phase != 0) {
      channel_states[5].cycles++;
      asyncStep_buzzer<5>();
    } else if(channel_states[6].del == 0 && channel_states[6].phase != 0) {
      channel_states[6].cycles++;
      asyncStep_hdd<6>();
    } else if(channel_states[7].del == 0 && channel_states[7].phase != 0) {
      channel_states[7].cycles++;
      asyncStep_hdd<7>();
    } else {
      phaseCorrupt = false;
    }
  }
  if(channel_states[0].del != 0) {
    channel_states[0].cycles++;
    asyncStep_customdrvfloppy<0>();
  }
  if(channel_states[1].del != 0) {
    channel_states[1].cycles++;
    asyncStep_stdfloppy<1>();
  }
  if(channel_states[2].del != 0) {
    channel_states[2].cycles++;
    asyncStep_stdfloppy<2>();
  }
  if(channel_states[3].del != 0) {
    channel_states[3].cycles++;
    asyncStep_stdfloppy<3>();
  }
  if(channel_states[4].del != 0) {
    channel_states[4].cycles++;
    asyncStep_transf<4>();
  }
  if(channel_states[5].del != 0) {
    channel_states[5].cycles++;
    asyncStep_buzzer<5>();
  }
  if(channel_states[6].del != 0) {
    channel_states[6].cycles++;
    asyncStep_hdd<6>();
  }
  if(channel_states[7].del != 0) {
    channel_states[7].cycles++;
    asyncStep_hdd<7>();
  }
}
#pragma GCC pop_options

//t in ms
void startTone(int ch, float f, u8 v) {
  if(channel_states[ch].del == 0) {
    switch(channel_cfgs[ch].c_t) {
      case CHANNEL_TYPE_CUSTOM_DRV_FLOPPY:
        if(channel_states[ch].del == 0) {
          channel_states[ch].cycles = 0;
        }
        while(channel_states[ch].phase !=0) {}
        channel_states[ch].del = freqToDelayTimerPer(f);
        channel_states[ch].delA = channel_states[ch].del*(1.0-PWM_DUTY);
        channel_states[ch].delB = channel_states[ch].del*(PWM_DUTY/2.0);
        break;
      case CHANNEL_TYPE_STD_FLOPPY:
        digitalWrite(channel_cfgs[ch].pinA, false);
        channel_states[ch].t_shutdown = 0;
        if(channel_states[ch].del == 0) {
          channel_states[ch].cycles = 0;
        }
        while(channel_states[ch].phase !=0) {}
        channel_states[ch].del = freqToDelayTimerPer(f);
        channel_states[ch].delA = channel_states[ch].del/2;
        channel_states[ch].delB = channel_states[ch].del/2;
        break;
      case CHANNEL_TYPE_BUZZER:
      case CHANNEL_TYPE_TRANSF:
        if(channel_states[ch].del == 0) {
          channel_states[ch].cycles = 0;
        }
        while(channel_states[ch].phase !=0) {}
        channel_states[ch].del = freqToDelayTimerPer(f);
        channel_states[ch].varA = v;
        channel_states[ch].delA = channel_states[ch].del*channel_states[ch].varA/128;
        channel_states[ch].delB = channel_states[ch].del*(128-channel_states[ch].varA)/128;
        break;
      case CHANNEL_TYPE_HDD:
        if(channel_states[ch].del == 0) {
          channel_states[ch].cycles = 0;
        }
        channel_states[ch].del = v * HDD_MAX_PULSE / (128*TIMER_PERIOD);
        break;
    }
  }
}

void stopTone(int ch) {
  if(channel_states[ch].del != 0) {
    switch(channel_cfgs[ch].c_t) {
      case CHANNEL_TYPE_CUSTOM_DRV_FLOPPY:
        if(channel_states[ch].phase !=0) {
          phaseCorrupt = true;
        }
        channel_states[ch].del = 0;
        break;
      case CHANNEL_TYPE_STD_FLOPPY:
        channel_states[ch].t_shutdown = millis();
        if(channel_states[ch].phase !=0) {
          phaseCorrupt = true;
        }
        channel_states[ch].del = 0;
        break;
      case CHANNEL_TYPE_BUZZER:
      case CHANNEL_TYPE_TRANSF:
        if(channel_states[ch].phase !=0) {
          phaseCorrupt = true;
        }
        channel_states[ch].del = 0;
        break;
      case CHANNEL_TYPE_HDD:
        //do nothing, hdd is self-resetting
        break;
    }
  }
}

void autoStartTone(int ch, int note, int v) {
  double f = pow(2.0f, ((note-69)/12.0f))*440.0;
  if(channel_states[ch].playingNote != note) {
    if((channel_states[ch].del == 0 || channel_states[ch].remapped) && f > channel_cfgs[ch].minFr && f < channel_cfgs[ch].maxFr) {
      //no remapping required
      channel_states[ch].playingNote = note;
      channel_states[ch].remapped = false;
      startTone(ch, f, v);
    } else {
      //try to remap note
      for(int i = 0; i < CHANNELS_COUNT; i++) {
        if(channel_cfgs[i].c_t != CHANNEL_TYPE_HDD && channel_states[i].del == 0 && f > channel_cfgs[i].minFr && f < channel_cfgs[i].maxFr) {
          channel_states[i].playingNote = note;
          channel_states[i].remapped = true;
          startTone(i, f, v);
          break;
        }
      }
    }
  }
}

void autoStopTone(int ch, int note) {
  if((channel_states[ch].del != 0 || channel_cfgs[ch].c_t == CHANNEL_TYPE_HDD) && !channel_states[ch].remapped && channel_states[ch].playingNote == note) {
    //no remapping
    stopTone(ch);
    channel_states[ch].playingNote = 0;
  } else {
    //try to find remapped channel
    for(int i = 0; i < CHANNELS_COUNT; i++) {
      if(channel_cfgs[i].c_t != CHANNEL_TYPE_HDD && channel_states[i].del != 0 && channel_states[i].remapped && channel_states[i].playingNote == note) {
        stopTone(i);
        channel_states[i].remapped = false;
        channel_states[i].playingNote = 0;
        break;
      }
    }
  }
}

void resetChannel(int ch) {
  noInterrupts(); //Disable timer
  stopTone(ch);
  channel_states[ch].playingNote = 0;
  channel_states[ch].remapped = false;
  switch(channel_cfgs[ch].c_t) {
    case CHANNEL_TYPE_STD_FLOPPY:
      channel_states[ch].del = 2500/TIMER_PERIOD;
      channel_states[ch].steps = 0;
      channel_states[ch].phase = 0;
      digitalWrite(channel_cfgs[ch].pinA, false);
      channel_states[ch].dir = true;
      digitalWrite(channel_cfgs[ch].pinB, channel_states[ch].dir);
      for(int i = 0; i < NUM_STEPS + 1; i++) {
        syncStep(ch);
      }
      digitalWrite(channel_cfgs[ch].pinA, true);
      channel_states[ch].del = 0;
      break;
    case CHANNEL_TYPE_CUSTOM_DRV_FLOPPY:
      channel_states[ch].del = 2500/TIMER_PERIOD;
      channel_states[ch].varA = 0;
      channel_states[ch].steps = 0;
      channel_states[ch].phase = 0;
      digitalWrite(channel_cfgs[ch].pinA, false);
      channel_states[ch].dir = false;
      for(int i = 0; i < NUM_STEPS + 1; i++) {
        syncStep(ch);
      }
      digitalWrite(channel_cfgs[ch].pinA, false);
      channel_states[ch].del = 0;
      channel_states[ch].varA = 0;
      channel_states[ch].steps = 0;
      channel_states[ch].phase = 0;
      break;
    case CHANNEL_TYPE_BUZZER:
    case CHANNEL_TYPE_HDD:
    case CHANNEL_TYPE_TRANSF:
      channel_states[ch].del = 0;
      channel_states[ch].varA = 0;
      channel_states[ch].phase = 0;
      digitalWrite(channel_cfgs[ch].pinA, false);
      break;
  }
  interrupts();
}

void processMidiUsb() {
  USBMIDI.poll();
  while (USBMIDI.available()) {
    u8 b = USBMIDI.read();
    midi_event_t evt;
    if(mtou.process(b, evt)) {
      switch(evt.m_event) { //determine midi command
        case 0b1000: {//note off
          u8 channelN = evt.m_data[0] & 0b1111;
          u8 k = evt.m_data[1];
          u8 v = evt.m_data[2];
          if(channelN > CHANNELS_COUNT - 1) {
            break;
          }
          autoStopTone(channelN, k);
          break;
        }
        case 0b1001: { //note on
          u8 channelN = evt.m_data[0] & 0b1111;
          u8 k = evt.m_data[1];
          u8 v = evt.m_data[2];
          if(channelN > CHANNELS_COUNT - 1) {
            break;
          }
          autoStartTone(channelN, k, v);
          break;
        }
        case 0b1011: { //channel message
          u8 channelN = evt.m_data[0] & 0b1111;
          u8 c = evt.m_data[1];
          u8 v = evt.m_data[2];
          if(channelN > CHANNELS_COUNT - 1) {
            break;
          }
          if(c == 120 && v == 0) {
            resetChannel(channelN);
          } else if(c == 123 && v == 0) {
            resetChannel(channelN);
          }
          break;
        }
      }
    }
  }
}

void autoShutdownchannel_states() {
  for(int i = 0; i < CHANNELS_COUNT; i++) {
    if(channel_cfgs[i].c_t == CHANNEL_TYPE_STD_FLOPPY) {
      if(channel_states[i].t_shutdown != 0) {
        if(millis() - channel_states[i].t_shutdown > 200) {
          digitalWrite(channel_cfgs[i].pinA, true);
          channel_states[i].t_shutdown = 0;
        }
      }
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  for(int i = 0; i < CHANNELS_COUNT; i++) {
    switch(channel_cfgs[i].c_t) {
      case CHANNEL_TYPE_STD_FLOPPY:
      case CHANNEL_TYPE_CUSTOM_DRV_FLOPPY:
        pinMode(channel_cfgs[i].pinA, OUTPUT);
        pinMode(channel_cfgs[i].pinB, OUTPUT);
        pinMode(channel_cfgs[i].pinC, OUTPUT);
        resetChannel(i);
        break;
      case CHANNEL_TYPE_BUZZER:
      case CHANNEL_TYPE_HDD:
      case CHANNEL_TYPE_TRANSF:
        pinMode(channel_cfgs[i].pinA, OUTPUT);
        resetChannel(i);
        break;
    }
  }
  Timer1.initialize(TIMER_PERIOD);
  Timer1.attachInterrupt(mainchannel_statesLoop);
}

void loop() {
  processMidiUsb();
  autoShutdownchannel_states();
}
