#include <iostream>
#include <gpiod.hpp>
#include <thread>
#include <chrono>
#include <map>
#include <string>
#include <algorithm>
#include <string_view>
#include <signal.h>
#include <alsa/asoundlib.h>
#include <mutex>
#include <math.h>

//HARDCODED FOR 8 CHANNELS: 4 FLOPPY DRIVES, TRANSFORMER, BUZZER, SMALL HDD, BIG HDD
//PINOUT(GPIO HEADER FOR ORANGE PI 3 LTS):
//PIN_DS = L2 = C0, 2
//PIN_STCP = L3 = C0, 3
//PIN_SHCP = D18 = C1, 114
//0=EN1 1=DIR1 2=EN2 3=DIR2 4=EN3 5=DIR3 6=EN4 7=DIR4(using shift register 74hc595)
//PIN_STEP1 = D15 = C1, 111
//PIN_STEP2 = D16 = C1, 112
//PIN_STEP3 = D21 = C1, 117
//PIN_STEP4 = H3 = C1, 227
//PIN_TRANSF = L8 = C0, 8
//PIN_BUZZER = H4 = C1, 228
//PIN_SMALLHDD = H6 = C1, 230
//PIN_BIGHDD = H5 = C1, 229

gpiod::chip chip0;
gpiod::chip chip1;
gpiod::line_bulk c0lines;
gpiod::line_bulk c1lines;
std::mutex gpiomtx; //gpio&some channel info mtx
bool working = true;
bool remappingenabled = false;
bool verbose = false;
std::thread sequencer_thread;
std::vector<int> chip0_state = {0, 0, 0};  //DS, STCP, TRANSF
std::vector<int> chip1_state = {0, 0, 0, 0, 0, 0, 0, 0}; //SHCP, STEP1, STEP2, STEP3, STEP4, BUZZER, SMALLHDD, BIGHDD
std::vector<int> shiftreg_state = {1, 0, 1, 1, 1, 1, 1, 1}; //EN1, DIR1, EN2, DIR2, EN3, DIR3, EN4, DIR4

snd_seq_t *midi_input_seq_handle;
int midi_input_port;

struct channel_cfg {
    //At the moment pins are hardcoded );
    int steps_count;
    float min_frequency;
    float max_frequency;
};

struct channel_state {
    int curr_steps;
    int curr_phase;
    int curr_playing_note;
    int curr_velocity;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_time;
    std::chrono::high_resolution_clock::duration curr_period;
    bool remapped;
    bool enabled;
};

#define CH_NUM 8

const channel_cfg channel_cfgs[CH_NUM] = {
    {160, 20.0, 525.0},
    {80, 20.0, 525.0},
    {80, 20.0, 525.0},
    {80, 20.0, 525.0},
    {0,  20.0, 300.0},
    {0,  50.0, 10000.0},
    {0,  0.0,  0.0},
    {0,  0.0,  0.0},
};

channel_state channel_states[CH_NUM];

// trim from end (in place)
static inline void rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
        return !std::isspace(ch);
    }).base(), s.end());
}

static inline double midiNoteToFrequency(float note) {
    return pow(2.0f, ((note-69)/12.0f))*440.0;
}

void update_shiftreg() {
    chip1_state[0] = 0; //SHCP=0
    c1lines.set_values(chip1_state);
    chip0_state[0] = 0; //DS=0
    chip0_state[1] = 0; //STCP=0
    c0lines.set_values(chip0_state);
    for(int i = 7; i >= 0; i--) {
        chip0_state[0] = shiftreg_state[i]; //DS=data
        c0lines.set_values(chip0_state);
        chip1_state[0] = 1; //SHCP=1
        c1lines.set_values(chip1_state);
        //delay, if required
        chip1_state[0] = 0; //SHCP=0
        c1lines.set_values(chip1_state);
    }
    chip0_state[0] = 0; //DS=0
    chip0_state[1] = 1; //STCP=1
    c0lines.set_values(chip0_state);
    //delay, if required
    chip0_state[1] = 0; //STCP=0
    c0lines.set_values(chip0_state);
}

void reset_channels() {
    gpiomtx.try_lock();
    for(int i = 0; i<8; i++) {
        channel_states[i].curr_steps = 0;
        channel_states[i].last_time = std::chrono::high_resolution_clock::now();
        channel_states[i].curr_period = (std::chrono::duration<long, std::micro> {0});
        channel_states[i].remapped = false;
        channel_states[i].enabled = false;
        channel_states[i].curr_playing_note = 0;
        channel_states[i].curr_velocity = 0;
    }
    shiftreg_state[1] = 0; //DRIVE1 with A4988 driver, direction inverted
    shiftreg_state[3] = 1;
    shiftreg_state[5] = 1;
    shiftreg_state[7] = 1; //DIR1=4=1(back)
    shiftreg_state[0] = 0;
    shiftreg_state[2] = 0;
    shiftreg_state[4] = 0;
    shiftreg_state[6] = 0; //EN1-4=0(enabled)
    update_shiftreg();
    std::this_thread::sleep_for(std::chrono::microseconds(500));
    //F=500 Hz
    for(int i = 0; i<170; i++) {
        chip1_state[1] = 1;
        chip1_state[2] = 1;
        chip1_state[3] = 1;
        chip1_state[4] = 1; //STEP1-4=1
        c1lines.set_values(chip1_state);
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
        chip1_state[1] = 0;
        chip1_state[2] = 0;
        chip1_state[3] = 0;
        chip1_state[4] = 0; //STEP1-4=0
        c1lines.set_values(chip1_state);
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }
    shiftreg_state[1] = 1; //DRIVE1 with A4988 driver, direction inverted
    shiftreg_state[3] = 0;
    shiftreg_state[5] = 0;
    shiftreg_state[7] = 0; //DIR1=4=0(fwd)
    shiftreg_state[0] = 1;
    shiftreg_state[2] = 1;
    shiftreg_state[4] = 1;
    shiftreg_state[6] = 1; //EN1-4=1(disabled)
    update_shiftreg();
    chip0_state[2] = 0; //reset transformer
    c0lines.set_values(chip0_state);
    chip1_state[5] = 0; //reset buzzer
    chip1_state[6] = 0; //reset smallhdd
    chip1_state[7] = 0; //reset bighdd
    c1lines.set_values(chip1_state);
    gpiomtx.unlock();
}

void reset_channel(int num) {
    if(num >= 0 && num < 4) {
        //floppy drives
        gpiomtx.try_lock();

        channel_states[num].curr_steps = 0;
        channel_states[num].curr_phase = 0;
        channel_states[num].last_time = std::chrono::high_resolution_clock::now();
        channel_states[num].curr_period = (std::chrono::duration<long, std::micro> {0});
        channel_states[num].remapped = false;
        channel_states[num].enabled = false;
        channel_states[num].curr_playing_note = 0;
        channel_states[num].curr_velocity = 0;

        shiftreg_state[num*2+1] = (num == 0) ? 0 : 1; //Backward direction for drive(inverted for 0 with A4988 driver)
        shiftreg_state[num*2] = 0; //enable drive
        update_shiftreg();
        std::this_thread::sleep_for(std::chrono::microseconds(500));
        //F=400 Hz
        for(int i = 0; i<channel_cfgs[num].steps_count+10; i++) {
            chip1_state[1+num] = 1; //STEP=1
            c1lines.set_values(chip1_state);
            std::this_thread::sleep_for(std::chrono::microseconds(1000));
            chip1_state[1+num] = 0; //STEP=0
            c1lines.set_values(chip1_state);
            std::this_thread::sleep_for(std::chrono::microseconds(1000));
        }
        shiftreg_state[num*2+1] = (num == 0) ? 1 : 0; //Forward direction for drive(inverted for 0 with A4988 driver)
        shiftreg_state[num*2] = 1; //disable drive
        update_shiftreg();
        gpiomtx.unlock();
    } else if(num == 4) {
        //transformer
        gpiomtx.try_lock();

        channel_states[num].curr_steps = 0;
        channel_states[num].curr_phase = 0;
        channel_states[num].last_time = std::chrono::high_resolution_clock::now();
        channel_states[num].curr_period = (std::chrono::duration<long, std::micro> {0});
        channel_states[num].remapped = false;
        channel_states[num].enabled = false;
        channel_states[num].curr_playing_note = 0;
        channel_states[num].curr_velocity = 0;

        chip0_state[2] = 0; //reset transformer
        c0lines.set_values(chip0_state);

        gpiomtx.unlock();
    } else if(num == 5) {
        //buzzer
        gpiomtx.try_lock();

        channel_states[num].curr_steps = 0;
        channel_states[num].curr_phase = 0;
        channel_states[num].last_time = std::chrono::high_resolution_clock::now();
        channel_states[num].curr_period = (std::chrono::duration<long, std::micro> {0});
        channel_states[num].remapped = false;
        channel_states[num].enabled = false;
        channel_states[num].curr_playing_note = 0;
        channel_states[num].curr_velocity = 0;

        chip1_state[5] = 0; //reset buzzer
        c1lines.set_values(chip1_state);

        gpiomtx.unlock();
    }
    //hdds ignored
}

void sequencer_thread_func() {
    while(working) {
        bool updateshiftreg = false;
        bool updatechip0 = false;
        bool updatechip1 = false;
        gpiomtx.try_lock();
        for(int i = 0; i < CH_NUM; i++) {
            if((channel_states[i].curr_phase != 0 || channel_states[i].curr_period.count() != 0) && (std::chrono::high_resolution_clock::now() - channel_states[i].last_time >= channel_states[i].curr_period)) {
                switch(i) {
                    case 0:
                    case 1:
                    case 2:
                    case 3:
                        if(channel_states[i].curr_phase == 0) {
                            chip1_state[1+i] = 1; //STEP=1
                            updatechip1 = true;
                            channel_states[i].curr_phase = 1;
                        } else {
                            chip1_state[1+i] = 0; //STEP=0
                            updatechip1 = true;
                            channel_states[i].curr_steps++;
                            if(channel_states[i].curr_steps >= channel_cfgs[i].steps_count) {
                                shiftreg_state[i*2+1] = !shiftreg_state[i*2+1]; //reverse direction
                                updateshiftreg = true;
                                channel_states[i].curr_steps = 0;
                            }
                            channel_states[i].curr_phase = 0;
                        }
                        break;
                    case 4:
                        if(channel_states[i].curr_phase == 0) {
                            chip0_state[2] = 1; //set transformer to 1
                            updatechip0 = true;
                            channel_states[i].curr_phase = 1;
                        } else {
                            chip0_state[2] = 0; //set transformer to 0
                            updatechip0 = true;
                            channel_states[i].curr_phase = 0;
                        }
                        break;
                    case 5:
                        if(channel_states[i].curr_phase == 0) {
                            chip1_state[5] = 1; //set buzzer to 1
                            updatechip1 = true;
                            channel_states[i].curr_phase = 1;
                        } else {
                            chip1_state[5] = 0; //set buzzer to 0
                            updatechip1 = true;
                            channel_states[i].curr_phase = 0;
                        }
                        break;
                    case 6:
                    case 7:
                        if(channel_states[i].curr_phase == 0) {
                            chip1_state[i] = 1; //set hdd to 1
                            updatechip1 = true;
                            channel_states[i].curr_phase = 1;
                        } else {
                            chip1_state[i] = 0; //set hdd to 0
                            updatechip1 = true;
                            channel_states[i].enabled = false;
                            channel_states[i].curr_period = std::chrono::high_resolution_clock::duration(0);
                            channel_states[i].curr_phase = 0;
                            channel_states[i].remapped = 0;
                            channel_states[i].curr_velocity = 0;
                        }
                        break;
                }
                channel_states[i].last_time = std::chrono::high_resolution_clock::now();
            } else if(channel_states[i].curr_period.count() == 0 && channel_states[i].enabled && (std::chrono::high_resolution_clock::now() - channel_states[i].last_time >= std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(std::chrono::milliseconds(200)))) {
                //Drives disable timer(200ms)
                switch(i) {
                    case 0:
                    case 1:
                    case 2:
                    case 3:
                        shiftreg_state[i*2] = 1; //disable drive
                        updateshiftreg = true;
                        break;
                }
                channel_states[i].enabled = false;
            }
        }
        if(updateshiftreg)
            update_shiftreg();
        if(updatechip0)
            c0lines.set_values(chip0_state);
        if(updatechip1)
            c1lines.set_values(chip1_state);
        gpiomtx.unlock();
        bool sleep = true;
        for(int i = 0; i < CH_NUM; i++) {
            if(channel_states[i].enabled) {
                sleep = false;
                break;
            }
        }
        if(sleep) {
            std::this_thread::sleep_for(std::chrono::microseconds(1000)); //Save CPU activity when no channels are active
        }
    }
}

void sigint_handler(int sig) {
    printf("\nSIGINT, terminating\n");
    working = false;
    snd_seq_delete_simple_port(midi_input_seq_handle, midi_input_port);
    snd_seq_close(midi_input_seq_handle);
    std::chrono::time_point<std::chrono::high_resolution_clock>  msr_start, msr_end;
    msr_start = std::chrono::high_resolution_clock::now();
    reset_channels();
    msr_end = std::chrono::high_resolution_clock::now();
    printf("Channels reset in %ld us\n", std::chrono::duration_cast<std::chrono::microseconds>(msr_end - msr_start).count());
    msr_start = std::chrono::high_resolution_clock::now();
    sequencer_thread.join();
    msr_end = std::chrono::high_resolution_clock::now();
    printf("Sequencer thread joined in %ld us\n", std::chrono::duration_cast<std::chrono::microseconds>(msr_end - msr_start).count());
    exit(0);
}

void setup_gpio() {
    chip0 = gpiod::chip("gpiochip0");
    chip1 = gpiod::chip("gpiochip1");
    c0lines = chip0.get_lines({2, 3, 8});
    c1lines = chip1.get_lines({114, 111, 112, 117, 227, 228, 230, 229});
    c0lines.request({"CNAF", gpiod::line_request::DIRECTION_OUTPUT, 0}, chip0_state);
    c1lines.request({"CNAF", gpiod::line_request::DIRECTION_OUTPUT, 0}, chip1_state);
}

void setup_alsaseq() {
    snd_seq_open(&midi_input_seq_handle, "default", SND_SEQ_OPEN_INPUT, 0);
    snd_seq_set_client_name(midi_input_seq_handle, "CNAF");
    midi_input_port = snd_seq_create_simple_port(midi_input_seq_handle, "in", SND_SEQ_PORT_CAP_WRITE|SND_SEQ_PORT_CAP_SUBS_WRITE, SND_SEQ_PORT_TYPE_APPLICATION);
}

void alsa_subscribe_to(std::string portname) {
    rtrim(portname);
    snd_seq_client_info_t *cinfo;
    snd_seq_port_info_t *pinfo;
    snd_seq_client_info_alloca(&cinfo);
    snd_seq_port_info_alloca(&pinfo);

    snd_seq_client_info_set_client(cinfo, -1);
    while (snd_seq_query_next_client(midi_input_seq_handle, cinfo) >= 0) {
        /* reset query info */
        snd_seq_port_info_set_client(pinfo, snd_seq_client_info_get_client(cinfo));
        snd_seq_port_info_set_port(pinfo, -1);
        while (snd_seq_query_next_port(midi_input_seq_handle, pinfo) >= 0) {
            std::string pname = std::string(snd_seq_port_info_get_name(pinfo));
            rtrim(pname);
            if(portname == pname) {
                int clid = snd_seq_port_info_get_client(pinfo);
                int portid = snd_seq_port_info_get_port(pinfo);
                snd_seq_connect_from(midi_input_seq_handle, midi_input_port, clid, portid);
                printf("Connected to port %s(app %s)\n", pname.c_str(), snd_seq_client_info_get_name(cinfo));
                return;
            }
        }
    }
    printf("Warning: requested midi port(%s) was not found!\n", portname.c_str());
}

void set_channel(int ch, long period_us, int velocity) {
    while(channel_states[ch].curr_phase != 0) {}
    gpiomtx.try_lock();
    switch(ch) {
        case 0:
            //Drive 0 requires double frequency
            channel_states[ch].curr_period = std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(std::chrono::microseconds(period_us/4));
            if(!channel_states[ch].enabled) {
                channel_states[ch].last_time = std::chrono::high_resolution_clock::now();
                shiftreg_state[ch*2] = 0; //enable drive
                update_shiftreg();
                std::this_thread::sleep_for(std::chrono::microseconds(2000));
                channel_states[ch].enabled = true;
            }
            break;
        case 1:
        case 2:
        case 3:
            channel_states[ch].curr_period = std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(std::chrono::microseconds(period_us/2));
            if(!channel_states[ch].enabled) {
                channel_states[ch].last_time = std::chrono::high_resolution_clock::now();
                shiftreg_state[ch*2] = 0; //enable drive
                update_shiftreg();
                std::this_thread::sleep_for(std::chrono::microseconds(2000));
                channel_states[ch].enabled = true;
            }
            break;
        case 4:
            channel_states[ch].enabled = true;
            channel_states[ch].curr_period = std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(std::chrono::microseconds(period_us/2));
            break;
        case 5:
            channel_states[ch].enabled = true;
            channel_states[ch].curr_period = std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(std::chrono::microseconds(period_us/2));
            break;
        case 6:
        case 7:
            channel_states[ch].enabled = true;
            channel_states[ch].curr_period = std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(std::chrono::microseconds(velocity*550L));
            break;
    }
    gpiomtx.unlock();
}

void clear_channel(int ch) {
    gpiomtx.try_lock();
    switch(ch) {
        case 0:
        case 1:
        case 2:
        case 3:
            channel_states[ch].curr_period = std::chrono::high_resolution_clock::duration(0);
            break;
        case 4:
            channel_states[ch].enabled = false;
            channel_states[ch].curr_period = std::chrono::high_resolution_clock::duration(0);
            break;
        case 5:
            channel_states[ch].enabled = false;
            channel_states[ch].curr_period = std::chrono::high_resolution_clock::duration(0);
            break;
    }
    gpiomtx.unlock();
}

void play_note(int ch, int note, int velocity) {
    double f = midiNoteToFrequency(note);
    long period_us = (1000000L / f);
    if(ch >= 8 && ch != 9) {
        return;
    }
    if(ch != 6 && ch != 7 && ch != 9) {
        if((f >= channel_cfgs[ch].min_frequency && f <= channel_cfgs[ch].max_frequency) && (channel_states[ch].remapped || channel_states[ch].curr_playing_note == 0)) {
            if(verbose)
                printf("     Channel %d playing note %f\n", ch, f);
            set_channel(ch, period_us, velocity);
            channel_states[ch].curr_playing_note = note;
            channel_states[ch].curr_velocity = velocity;
            channel_states[ch].remapped = false;
        } else if(remappingenabled) {
            //remapping process
            bool remap = false;
            for(int i = 0; i < 6; i++) {
                if(channel_states[i].curr_playing_note == 0) {
                    if(f >= channel_cfgs[i].min_frequency && f <= channel_cfgs[i].max_frequency) {
                        if(verbose)
                            printf("     Channel %d remapping note %f to %d(currently playing %d)\n", ch, f, i, channel_states[ch].curr_playing_note);
                        set_channel(i, period_us, velocity);
                        channel_states[i].curr_playing_note = note;
                        channel_states[ch].curr_velocity = velocity;
                        channel_states[i].remapped = true;
                        remap = true;
                        break;
                    }
                }
            }
            if(!remap) {
                if((f >= channel_cfgs[ch].min_frequency && f <= channel_cfgs[ch].max_frequency)) {
                    if(verbose)
                        printf("     Channel %d overwriting note %f\n", ch, f);
                    set_channel(ch, period_us, velocity);
                    channel_states[ch].curr_playing_note = note;
                    channel_states[ch].curr_velocity = velocity;
                    channel_states[ch].remapped = false;
                }
            }
        }
    } else if(ch == 9) {
        switch(note) {
            case 38: //Acoustic Snare
            case 39: //Hand Clap
            case 40: //Electric Snare
            case 42: //Closed Hi Hat 
            case 44: //Pedal Hi-Hat 
            case 46: //Open Hi-Hat 
            case 49: //Crash Cymbal 1
            case 51: //Ride Cymbal 1
            case 52: //Chinese Cymbal
            case 53: //Ride Bell
            case 55: //Splash Cymbal
            case 56: //Cowbell
            case 57: //Crash Cymbal 2
            case 58: //Vibraslap
            case 59: //Ride Cymbal 2
            case 70: //Maracas
            case 71: //Short Whistle
            case 72: //Long Whistle
            case 76: //Hi Wood Block
            case 77: //Low Wood Block
            case 80: //Mute Triangle
            case 81: //Open Triangle
                //higher drums or cymbals/hats
                set_channel(6, period_us, velocity);
                break;
            default:
                //drums or other
                set_channel(7, period_us, velocity);
                break;
        }
    } else {
        set_channel(ch, period_us, velocity);
    }
}

void stop_note(int ch, int note) {
    if(ch >= 8 && ch != 9) {
        return;
    }
    if(channel_states[ch].curr_playing_note == note && !channel_states[ch].remapped) {
        clear_channel(ch);
        channel_states[ch].curr_playing_note = 0;
        channel_states[ch].curr_velocity = 0;
        channel_states[ch].remapped = false;
    } else if(remappingenabled) {
        //disable remapped note
        for(int i = 0; i < 6; i++) {
                if(channel_states[i].curr_playing_note == note && channel_states[i].remapped) {
                    clear_channel(i);
                    channel_states[i].curr_playing_note = 0;
                    channel_states[ch].curr_velocity = 0;
                    channel_states[i].remapped = false;
                    break;
                }
            }
    }
}

void pitch_bend(int ch, int bend) {
    if(ch != 6 && ch != 7) {
        if(channel_states[ch].curr_playing_note != 0 && !channel_states[ch].remapped) {
            float semitone_bend = bend/4096.0f;
            double f = midiNoteToFrequency(channel_states[ch].curr_playing_note+semitone_bend);;
            long period_us = (1000000L / f);
            set_channel(ch, period_us, channel_states[ch].curr_velocity);
        }
    }
}

void main_loop() {
    snd_seq_event_t *evt;
    snd_seq_event_input(midi_input_seq_handle, &evt);
    if(verbose)
        printf("Midi evt -> Type: %d Note: %d Vel: %d Chn: %d Chp: %d Param: %d Val: %d\n", evt->type, evt->data.note.note, evt->data.note.velocity, evt->data.note.channel, evt->data.control.channel, evt->data.control.param, evt->data.control.value);
    switch(evt->type) {
        case SND_SEQ_EVENT_NOTEON:
            play_note(evt->data.note.channel, evt->data.note.note, evt->data.note.velocity);
            break;
        case SND_SEQ_EVENT_NOTEOFF:
            stop_note(evt->data.note.channel, evt->data.note.note);
            break;
        case SND_SEQ_EVENT_CONTROLLER:
            if(evt->data.control.param == 123 && evt->data.control.value == 0) {
                if(verbose)
                        printf("     Channel %d reset\n", evt->data.control.channel);
                reset_channel(evt->data.control.channel);
            }
            break;
        case SND_SEQ_EVENT_PITCHBEND:
            pitch_bend(evt->data.control.channel, evt->data.control.value);
            break;
    }
}

void print_help() {
    printf("CertainlyNotAFloppotron program (CL) indir, 2022(GPL)\n");
    printf("./cnaf [args]\n");
    printf("--midiport (PORT)  Subscribe to an midi port by name(list by aconnect -l)\n");
    printf("--allowremapping   Allow channels remapping\n");
    printf("--verbose          Verbose output\n");
}

int main(int argc, char** argv) {
    printf("\n\n");
    printf(" ██████ ███    ██  █████  ███████ \n");
    printf("██      ████   ██ ██   ██ ██      \n");
    printf("██      ██ ██  ██ ███████ █████   \n");
    printf("██      ██  ██ ██ ██   ██ ██      \n");
    printf(" ██████ ██   ████ ██   ██ ██      \n\n\n");
    std::map<std::string, std::string> parameters;
    for(int i = 1; i < argc; i++) {
        std::string arg = std::string(argv[i]);
        if(arg == "--help") {
            print_help();
            return 1;
        } else if(arg == "--verbose") {
            verbose = true;
        } else if(arg == "--allowremapping") {
            remappingenabled = true;
        } else if(arg.starts_with("--") && (i+1 < argc)) {
            std::string name = arg.substr(2, arg.size()-2);
            i++;
            std::string val = std::string(argv[i]);
            parameters.insert(std::pair<std::string,std::string>(name, val));
        } else {
            parameters.insert(std::pair<std::string,std::string>(arg, "true"));
        }
    }
    signal(SIGINT, sigint_handler);
    std::chrono::time_point<std::chrono::high_resolution_clock>  msr_start, msr_end;
    printf("Starting CNAF program...\n");
    msr_start = std::chrono::high_resolution_clock::now();
    setup_gpio();
    msr_end = std::chrono::high_resolution_clock::now();
    printf("Got GPIO lines in %ld us\n", std::chrono::duration_cast<std::chrono::microseconds>(msr_end - msr_start).count());
    msr_start = std::chrono::high_resolution_clock::now();
    update_shiftreg();
    msr_end = std::chrono::high_resolution_clock::now();
    printf("Shift register reset in %ld us\n", std::chrono::duration_cast<std::chrono::microseconds>(msr_end - msr_start).count());
    msr_start = std::chrono::high_resolution_clock::now();
    reset_channels();
    msr_end = std::chrono::high_resolution_clock::now();
    printf("Channels reset in %ld us\n", std::chrono::duration_cast<std::chrono::microseconds>(msr_end - msr_start).count());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    setup_alsaseq();
    if(parameters.find("midiport") != parameters.end()) {
        std::string portname = parameters["midiport"];
        alsa_subscribe_to(portname);
    }

    sequencer_thread = std::thread(sequencer_thread_func);

    while(working) {
        main_loop();
    }

    return 0;
}
