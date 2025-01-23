#pragma once

#include <Arduino.h>
#include <stdbool.h>
#include <stdint.h>

// comment out the data you are not interested, but at least leave one row !
const uint8_t opdata[][2] PROGMEM = {
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
  FRAME_VALID                =  0,
  ERR_MSG_INVALID_SIGNATURE  = -1,
  ERR_MSG_INVALID_CHECKSUM   = -2,
  ERR_MSG_TIMEOUT_CLOCK_LOW  = -3,
  ERR_MSG_TIMEOUT_CLOCK_HIGH = -4,
};

enum ACType {  // Type enum
  ACTYPE_STATUS    = 0x40,
  ACTYPE_OPDATA    = 0x80,
  ACTYPE_ERROPDATA = 0xc0,
};

enum ACStatus {  // Status enum
  ACSTATUS_POWER = ACTYPE_STATUS,
  ACSTATUS_MODE,
  ACSTATUS_FAN,
  ACSTATUS_VANES_HORIZONTAL,
  ACSTATUS_VANES_VERTICAL,
  ACSTATUS_3DAUTO,
  ACSTATUS_TROOM,
  ACSTATUS_TSETPOINT,
  ACSTATUS_ERRORCODE,

  ACSTATUS_OPDATA_MODE = ACTYPE_OPDATA,
  ACSTATUS_OPDATA_kWh,
  ACSTATUS_OPDATA_INDOOR_TEMPERATURE,
  ACSTATUS_OPDATA_INDOOR_TEMPERATURE_RETURN_AIR,
  ACSTATUS_OPDATA_OUTDOOR_TEMPERATURE,
  ACSTATUS_OPDATA_OUTDOOR_TEMPERATURE_HEAT_EXCHANGER,
  ACSTATUS_OPDATA_INDOOR_FANSPEED,
  ACSTATUS_OPDATA_INDOOR_TEMPERATURE_UBEND,
  ACSTATUS_OPDATA_INDOOR_TEMPERATURE_CAPILLARY,
  ACSTATUS_OPDATA_INDOOR_TEMPERATURE_SUCTION_HEADER,
  ACSTATUS_OPDATA_OUTDOOR_FANSPEED,
  ACSTATUS_OPDATA_INDOOR_TOTAL_RUNTIME,
  ACSTATUS_OPDATA_TOTAL_COMPRESSOR_RUNTIME,
  ACSTATUS_OPDATA_COMPRESSOR_FREQUENCY,
  ACSTATUS_OPDATA_CT_CURRENT,
  ACSTATUS_OPDATA_OUTDOOR_TEMPERATURE_D,
  ACSTATUS_OPDATA_OUTDOOR_TEMPERATURE_DSH,
  ACSTATUS_OPDATA_PROTECTION_NO,
  ACSTATUS_OPDATA_DEFROST,
  ACSTATUS_OPDATA_OUTDOOR_EEV1,
  ACSTATUS_OPDATA_UNKNOWN,

  ACSTATUS_ERR_OPDATA_MODE = ACTYPE_ERROPDATA,
  ACSTATUS_ERR_OPDATA_INDOOR_TEMPERATURE,
  ACSTATUS_ERR_OPDATA_INDOOR_TEMPERATURE_RETURN_AIR,
  ACSTATUS_ERR_OPDATA_INDOOR_TEMPERATURE_UBEND,
  ACSTATUS_ERR_OPDATA_INDOOR_TEMPERATURE_CAPILLARY,
  ACSTATUS_ERR_OPDATA_INDOOR_TEMPERATURE_SUCTION_HEADER,
  ACSTATUS_ERR_OPDATA_INDOOR_FANSPEED,
  ACSTATUS_ERR_OPDATA_INDOOR_TOTAL_RUNTIME,
  ACSTATUS_ERR_OPDATA_OUTDOOR_TEMPERATURE,
  ACSTATUS_ERR_OPDATA_OUTDOOR_TEMPERATURE_HEAT_EXCHANGER,
  ACSTATUS_ERR_OPDATA_COMPRESSOR_FREQUENCY,
  ACSTATUS_ERR_OPDATA_OUTDOOR_TEMPERATURE_D,
  ACSTATUS_ERR_OPDATA_CT_CURRENT,
  ACSTATUS_ERR_OPDATA_OUTDOOR_FANSPEED,
  ACSTATUS_ERR_OPDATA_TOTAL_COMPRESSOR_RUNTIME,
  ACSTATUS_ERR_OPDATA_OUTDOOR_EEV1,
  ACSTATUS_ERR_OPDATA_ERRORCODE,
};

enum ACPower {  // Power enum
  ACPOWER_OFF = 0,
  ACPOWER_ON  = 1,
};

enum ACMode {  // Mode enum
  ACMODE_AUTO = 0b00000000,
  ACMODE_DRY  = 0b00000100,
  ACMODE_COOL = 0b00001000,
  ACMODE_FAN  = 0b00001100,
  ACMODE_HEAT = 0b00010000,
};

enum ACFan {  // FAN speed enum
  ACFAN_QUIET  = 0b00000000,
  ACFAN_LOW    = 0b00000001,
  ACFAN_MEDIUM = 0b00000010,
  ACFAN_HIGH   = 0b00000110,
  ACFAN_AUTO   = 0b00000111,
};

enum ACVanesHorizontal {  // Vanes enum
  ACVANES_HORIZONTAL_UNKNOWN = 0,
  ACVANES_HORIZONTAL_1       = 1,
  ACVANES_HORIZONTAL_2       = 2,
  ACVANES_HORIZONTAL_3       = 3,
  ACVANES_HORIZONTAL_4       = 4,
  ACVANES_HORIZONTAL_SWING   = 5,
};

enum ACVanesVertical {  // Vanes Left Right enum
  ACVANES_VERTICAL_1     = 1,
  ACVANES_VERTICAL_2     = 2,
  ACVANES_VERTICAL_3     = 3,
  ACVANES_VERTICAL_4     = 4,
  ACVANES_VERTICAL_5     = 5,
  ACVANES_VERTICAL_6     = 6,
  ACVANES_VERTICAL_7     = 7,
  ACVANES_VERTICAL_SWING = 8,
};

enum AC3Dauto {  // 3D auto enum
  ACVANES_3DAUTO_OFF = 0b00000000,
  ACVANES_3DAUTO_ON  = 0b00000100,
};

class CallbackInterfaceStatus {
  public:
    virtual void status_cb(ACStatus status, int value) = 0;
};

class MHI_AC_Ctrl_Core {
  private:
    // old status
    uint8_t status_power_old;
    uint8_t status_mode_old;
    uint8_t status_fan_old;
    uint8_t status_vanes_horizontal_old;
    uint8_t status_troom_old;
    uint8_t status_tsetpoint_old;
    uint8_t status_errorcode_old;

    uint8_t status_vanes_vertical_old;
    uint8_t status_3dauto_old;

    // old operating data
    uint16_t op_kwh;
    uint8_t op_mode;
    uint8_t op_indoor_temperature;
    uint8_t op_indoor_temperature_return_air;
    uint8_t op_indoor_fanspeed;
    uint8_t op_indoor_temperature_ubend;
    uint8_t op_indoor_temperature_capillary;
    uint8_t op_indoor_temperature_suction_header;
    uint8_t op_indoor_total_runtime;
    uint8_t op_outdoor_temperature;
    uint8_t op_outdoor_temperature_heat_exchanger;
    uint8_t op_outdoor_total_compressor_runtime;
    uint8_t op_outdoor_ct_current;
    uint8_t op_outdoor_temperature_dsh;
    uint8_t op_outdoor_protection_no;
    uint8_t op_outdoor_fanspeed;
    uint8_t op_outdoor_defrost;
    uint16_t op_outdoor_compressor_frequency;
    uint8_t op_outdoor_temperature_d;
    uint16_t op_outdoor_eev1;

    // for writing to AC
    uint8_t power_new = 0;
    uint8_t mode_new = 0;
    uint8_t tsetpoint_new = 0;
    uint8_t fan_new = 0;
    uint8_t vanes_horizontal_0_new = 0;
    uint8_t vanes_horizontal_1_new = 0;
    bool error_operating_data = false;
    uint8_t troom_new = 0xff;  // writing 0xff to DB3 indicates the usage of the internal room temperature sensor
    float troom_offset = 0.0;

    uint8_t vanes_vertical_0_new = 0;
    uint8_t vanes_vertical_1_new = 0;
    uint8_t vanes_3dauto_new = 0;
    uint8_t framesize = 20;

    CallbackInterfaceStatus *status_cb;

  public:
    void MHI_AC_ctrl_status(CallbackInterfaceStatus *cb) {
      status_cb = cb;
    };

    void init();                          // initialization called once after boot
    void reset_old_values();              // resets the 'old' variables ensuring that all status information are resend
    int loop(uint32_t max_time_ms);       // receive / transmit a frame of 20 uint8_ts
    void set_power(bool power);        // power on/off the AC
    void set_mode(ACMode mode);           // change AC mode (e.g. heat, dry, cool etc.)
    void set_tsetpoint(uint tsetpoint);   // set the target temperature of the AC)
    void set_fan(uint fan);               // set the requested fan speed
    void set_fan(enum ACFan fan);         // set the requested fan speed
    void set_vanes_horizontal(uint vanes);  // set the vanes horizontal position (or swing)
    void set_troom(uint8_t temperature);     // set the room temperature used by AC (0xff indicates the usage of the internal room temperature sensor)
    void enable_error_operating_data();             // request that the AC provides the error data
    float get_troom_offset();             // get troom offset, only usefull when ENHANCED_RESOLUTION is used
    void set_troom_offset(float offset);  // set troom offset, only usefull when ENHANCED_RESOLUTION is used
    void set_frame_size(uint8_t framesize);  // set framesize to 20 or 33
    void set_3dauto(AC3Dauto vanes_3dauto); // set the 3D auto mode on or off
    void set_vanes_vertical(uint vanes);  // set the vanes LR (vertical) position
};
