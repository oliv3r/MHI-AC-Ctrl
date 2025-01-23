#pragma once

#include <Arduino.h>
#include <stdbool.h>
#include <stdint.h>

// comment out the data you are not interested, but at least leave one row !
const byte opdata[][2] PROGMEM = {
  { 0xc0, 0x02 },  //  1 "MODE"
  { 0xc0, 0x05 },  //  2 "SET-TEMP" [°C]
  { 0xc0, 0x80 },  //  3 "RETURN-AIR" [°C]
  { 0xc0, 0x81 },  //  5 "THI-R1" [°C]
  { 0x40, 0x81 },  //  6 "THI-R2" [°C]
  { 0xc0, 0x87 },  //  7 "THI-R3" [°C]
  { 0xc0, 0x1f },  //  8 "IU-FANSPEED"
  { 0xc0, 0x1e },  // 12 "TOTAL-IU-RUN" [h]
  { 0x40, 0x80 },  // 21 "OUTDOOR" [°C]
  { 0x40, 0x82 },  // 22 "THO-R1" [°C]
  { 0x40, 0x11 },  // 24 "COMP" [Hz]
  { 0x40, 0x85 },  // 27 "TD" [°C]
  { 0x40, 0x90 },  // 29 "CT" [A]
  { 0x40, 0xb1 },  // 32 "TDSH" [°C]
  { 0x40, 0x7c },  // 33 "PROTECTION-No"
  { 0x40, 0x1f },  // 34 "OU-FANSPEED"
  { 0x40, 0x0c },  // 36 "DEFROST"
  { 0x40, 0x1e },  // 37 "TOTAL-COMP-RUN" [h]
  { 0x40, 0x13 },  // 38 "OU-EEV" [Puls]
  { 0xc0, 0x94 },  //    "energy-used" [kWh]
};

//#define NoFramesPerPacket 20      // number of frames/packet, must be an even number
#define NoFramesPerOpDataCycle 400  // number of frames used for a OpData request cycle; will be 20s (20 frames are 1s)
#define minTimeInternalTroom 5000   // minimal time in ms used for Troom internal sensor changes for publishing to avoid jitter

// pin defintions
#define SCK_PIN  14
#define MOSI_PIN 13
#define MISO_PIN 12

// constants for the frame
#define SB0 0
#define SB1  SB0 + 1
#define SB2  SB0 + 2

#define DB0  SB2 + 1
#define DB1  SB2 + 2
#define DB2  SB2 + 3
#define DB3  SB2 + 4
#define DB4  SB2 + 5
#define DB6  SB2 + 7
#define DB9  SB2 + 10
#define DB10 SB2 + 11
#define DB11 SB2 + 12
#define DB12 SB2 + 13
#define DB14 SB2 + 15

#define CBH  DB14 + 1
#define CBL  DB14 + 2

#define DB15 CBL  + 1
#define DB16 CBL  + 2
#define DB17 CBL  + 3
#define DB18 CBL  + 4
#define DB19 CBL  + 5
#define DB20 CBL  + 6
#define DB21 CBL  + 7
#define DB22 CBL  + 8
#define DB23 CBL  + 9
#define DB24 CBL  + 10
#define DB25 CBL  + 11
#define DB26 CBL  + 12

#define CBL2 DB26 + 1

enum ErrMsg {  // Error message enum
  err_msg_valid_frame       =  0,
  err_msg_invalid_signature = -1,
  err_msg_invalid_checksum  = -2,
  err_msg_timeout_SCK_low   = -3,
  err_msg_timeout_SCK_high  = -4,
};

enum ACType {  // Type enum
  type_status    = 0x40,
  type_opdata    = 0x80,
  type_erropdata = 0xc0,
};

enum ACStatus {  // Status enum
  status_power = type_status,
  status_mode,
  status_fan,
  status_vanes_horizontal,
  status_vanes_vertical,
  status_3dauto,
  status_troom,
  status_tsetpoint,
  status_errorcode,

  opdata_mode = type_opdata,
  opdata_kwh,
  opdata_tsetpoint,
  opdata_return_air,
  opdata_outdoor,
  opdata_tho_r1,
  opdata_iu_fanspeed,
  opdata_thi_r1,
  opdata_thi_r2,
  opdata_thi_r3,
  opdata_ou_fanspeed,
  opdata_total_iu_run,
  opdata_total_comp_run,
  opdata_comp,
  opdata_ct,
  opdata_td,
  opdata_tdsh,
  opdata_protection_no,
  opdata_defrost,
  opdata_ou_eev1,
  opdata_unknown,

  erropdata_mode = type_erropdata,
  erropdata_tsetpoint,
  erropdata_return_air,
  erropdata_thi_r1,
  erropdata_thi_r2,
  erropdata_thi_r3,
  erropdata_iu_fanspeed,
  erropdata_total_iu_run,
  erropdata_outdoor,
  erropdata_tho_r1,
  erropdata_comp,
  erropdata_td,
  erropdata_ct,
  erropdata_ou_fanspeed,
  erropdata_total_comp_run,
  erropdata_ou_eev1,
  erropdata_errorcode,
};

enum ACPower {  // Power enum
  power_off = 0,
  power_on  = 1,
};

enum ACMode {  // Mode enum
  mode_auto = 0b00000000,
  mode_dry  = 0b00000100,
  mode_cool = 0b00001000,
  mode_fan  = 0b00001100,
  mode_heat = 0b00010000,
};

enum ACVanesHorizontal {  // Vanes enum
  vanesHorizontal_unknown = 0,
  vanesHorizontal_1       = 1,
  vanesHorizontal_2       = 2,
  vanesHorizontal_3       = 3,
  vanesHorizontal_4       = 4,
  vanesHorizontal_swing   = 5,
};

enum ACVanesVertical {  // Vanes Left Right enum
  vanesVertical_1     = 1,
  vanesVertical_2     = 2,
  vanesVertical_3     = 3,
  vanesVertical_4     = 4,
  vanesVertical_5     = 5,
  vanesVertical_6     = 6,
  vanesVertical_7     = 7,
  vanesVertical_swing = 8,
};

enum AC3Dauto {  // 3D auto enum
  vanes_3dauto_off = 0b00000000,
  vanes_3dauto_on  = 0b00000100,
};

class CallbackInterface_Status {
  public:
    virtual void status_cb(ACStatus status, int value) = 0;
};

class MHI_AC_Ctrl_Core {
  private:
    // old status
    byte status_power_old;
    byte status_mode_old;
    byte status_fan_old;
    byte status_vanes_horizontal_old;
    byte status_troom_old;
    byte status_tsetpoint_old;
    byte status_errorcode_old;

    byte status_vanes_vertical_old;
    byte status_3dauto_old;

    // old operating data
    uint16_t op_kwh;
    byte op_mode;
    byte op_indoor_temperature;
    byte op_indoor_temperature_return_air;
    byte op_indoor_fanspeed;
    byte op_indoor_temperature_ubend;
    byte op_indoor_temperature_capillary;
    byte op_indoor_temperature_suction_header;
    byte op_indoor_total_runtime;
    byte op_outdoor_temperature;
    byte op_outdoor_temperature_heat_exchanger;
    byte op_outdoor_total_compressor_runtime;
    byte op_outdoor_ct_current;
    byte op_outdoor_temperature_dsh;
    byte op_outdoor_protection_no;
    byte op_outdoor_fanspeed;
    byte op_outdoor_defrost;
    uint16_t op_outdoor_compressor_frequency;
    byte op_outdoor_temperature_d;
    uint16_t op_outdoor_eev1;

    // for writing to AC
    byte power_new = 0;
    byte mode_new = 0;
    byte tsetpoint_new = 0;
    byte fan_new = 0;
    byte vanes_horizontal_0_new = 0;
    byte vanes_horizontal_1_new = 0;
    bool error_operating_data = false;
    byte troom_new = 0xff;  // writing 0xff to DB3 indicates the usage of the internal room temperature sensor
    float troom_offset = 0.0;

    byte vanes_vertical_0_new = 0;
    byte vanes_vertical_1_new = 0;
    byte vanes_3dauto_new = 0;
    byte framesize = 20;

    CallbackInterface_Status *status_cb;

  public:
    void MHI_AC_ctrl_status(CallbackInterface_Status *cb) {
      status_cb = cb;
    };

    void init();                          // initialization called once after boot
    void reset_old_values();              // resets the 'old' variables ensuring that all status information are resend
    int loop(uint max_time_ms);           // receive / transmit a frame of 20 bytes
    void set_power(bool power);           // power on/off the AC
    void set_mode(ACMode mode);           // change AC mode (e.g. heat, dry, cool etc.)
    void set_tsetpoint(uint tsetpoint);   // set the target temperature of the AC)
    void set_fan(uint fan);               // set the requested fan speed
    void set_vanes_horizontal(uint vanes);  // set the vanes horizontal position (or swing)
    void set_troom(byte temperature);     // set the room temperature used by AC (0xff indicates the usage of the internal room temperature sensor)
    void enable_error_operating_data();             // request that the AC provides the error data
    float get_troom_offset();             // get troom offset, only usefull when ENHANCED_RESOLUTION is used
    void set_troom_offset(float offset);  // set troom offset, only usefull when ENHANCED_RESOLUTION is used
    void set_frame_size(byte framesize);  // set framesize to 20 or 33
    void set_3dauto(AC3Dauto vanes_3dauto); // set the 3D auto mode on or off
    void set_vanes_vertical(uint vanes);  // set the vanes LR (vertical) position
};
