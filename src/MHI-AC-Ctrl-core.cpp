// MHI-AC-Ctrol-core
// implements the core functions (read & write SPI)

#include <Arduino.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "MHI-AC-Ctrl-core.h"

uint16_t calc_checksum(uint8_t *frame, size_t len) {
  uint16_t checksum = 0x0000;

  for (size_t i = 0; i < len; i++)
    checksum += frame[i];

  return checksum;
}

void MHI_AC_Ctrl_Core::reset_old_values() {  // used e.g. when MQTT connection to broker is lost, to re-output data
  // old status
  this->status_power_old = 0xff;
  this->status_mode_old = 0xff;
  this->status_fan_old = 0xff;
  this->status_vanes_horizontal_old = 0xff;
  this->status_troom_old = 0xfe;
  this->status_tsetpoint_old = 0x00;
  this->status_errorcode_old = 0xff;
  this->status_vanes_vertical_old = 0xff;
  this->status_3dauto_old = 0xff;

  // operating data
  this->op_kwh = 0xffff;
  this->op_mode = 0xff;
  this->op_indoor_temperature = 0xff;
  this->op_indoor_temperature_return_air = 0xff;
  this->op_indoor_fanspeed = 0xff;
  this->op_indoor_temperature_ubend = 0x00;
  this->op_indoor_temperature_capillary = 0x00;
  this->op_indoor_temperature_suction_header = 0x00;
  this->op_indoor_total_runtime = 0;
  this->op_outdoor_temperature = 0xff;
  this->op_outdoor_temperature_heat_exchanger = 0x00;
  this->op_outdoor_total_compressor_runtime = 0;
  this->op_outdoor_ct_current = 0xff;
  this->op_outdoor_temperature_dsh = 0xff;
  this->op_outdoor_protection_no = 0xff;
  this->op_outdoor_fanspeed = 0xff;
  this->op_outdoor_defrost = 0x00;
  this->op_outdoor_compressor_frequency = 0xffff;
  this->op_outdoor_temperature_d = 0x00;
  this->op_outdoor_eev1 = 0xffff;
}

void MHI_AC_Ctrl_Core::init() {
  pinMode(SCK_PIN, INPUT);
  pinMode(MOSI_PIN, INPUT);
  pinMode(MISO_PIN, OUTPUT);
  MHI_AC_Ctrl_Core::reset_old_values();
}

/*
 * wait_pin_  Wait for a *pin to change to *state and stay there for at least
 *            *state_ms. Error if *state is not reached within *timeout_ms.
 *
 * Returns 0 if pin has reached *state, negative error otherwise
 */
int HOT wait_pin_(uint8_t pin, uint8_t state,
                  uint32_t timeout_ms, uint32_t stable_state_ms) {
  uint32_t start_ms = millis();

  do {
    uint32_t pin_ms = 0;

    if ((millis() - start_ms) > timeout_ms)
      return -1;

    if (digitalRead(pin) == state)
      pin_ms = millis();

  } while ((millis() - pin_ms) <= stable_state_ms);

  return 0;
}

int MHI_AC_Ctrl_Core::loop(uint32_t max_time_ms) {
  const uint8_t opdata_cnt = sizeof(opdata) / sizeof(uint8_t) / 2;
  static uint8_t opdata_no = 0;              //
  uint32_t start_ms = millis();           // start time of this loop run
  uint8_t MOSI_byte;                        // received MOSI byte
  bool new_datapacket_received = false;  // indicated that a new frame was received
  static uint8_t erropdata_cnt = 0;          // number of expected error operating data
  static bool doubleframe = false;
  static int frame = 1;
  static uint8_t MOSI_frame[MHI_FRAME_SIZE_EXTENDED];
  //                            sb0   sb1   sb2   db0   db1   db2   db3   db4   db5   db6   db7   db8   db9  db10  db11  db12  db13  db14  chkH  chkL  db15  db16  db17  db18  db19  db20  db21  db22  db23  db24  db25  db26  chk2L
  static uint8_t MISO_frame[] = { 0xA9, 0x00, 0x07, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x22 };

  static uint32_t call_counter = 0;                     // counts how often this loop was called
  static uint32_t last_troom_interval_ms = 0; // remember when Troom internal has changed
  int ret = 0;

  if (this->framesize == MHI_FRAME_SIZE_EXTENDED)
    MISO_frame[0] = 0xAA;

  call_counter++;

/*
 * Wait for the SPI clock to become idle, indicating a frame pause. A frame
 * pause is about 40 ms and happens in between transmission of a frame.
 *
 * The frame pause thus indicates that a new frame is about to start and can be
 * used to sync transmissions against. It is reasonable to assume, that the
 * detection of the frame pause happens just after the last bit of the previous
 * frame has been sent, so after the frame pause detection, there's ample time
 * to do some processing.
 */
  ret = wait_pin_(SCK_pin, LOW, 5, millis() - max_time_ms - start_ms);
  if (ret)
    return err_msg_timeout_SCK_low;

  // build the next MISO frame
  doubleframe = !doubleframe;           // toggle every frame
  MISO_frame[DB14] = doubleframe << 2;  // MISO_frame[DB14] bit2 toggles with every frame

  // Requesting all different opdata's is an opdata cycle. A cycle will take 20s.
  // With the current 20 different opdata's, every opdata request will take 1sec (interval).
  // If there are only 5 different opdata's defined, these 5 will be spread about the 20s cycle. The interval will increase.
  // requesting a new opdata will always start at a doubleframe start
  if ((frame > (NoFramesPerOpDataCycle / opdata_cnt)) && doubleframe)  // interval for requesting new opdata depending on de number of opdata requests
    frame = 1;  // start requesting new OpData

  if (frame++ <= 2) {   // use opdata request only for 2 subsequent frames
    if (doubleframe) {  // start when MISO_frame[DB14] bit2 is set
      if (erropdata_cnt == 0) {
        MISO_frame[DB6] = opdata[opdata_no][0];
        MISO_frame[DB9] = opdata[opdata_no][1];
        opdata_no = (opdata_no + 1) % opdata_cnt;
      }

    }
  }
  else  // reset OpData request
  {
    MISO_frame[DB6] = 0x80;
    MISO_frame[DB9] = 0xff;
  }

  if (doubleframe) {  // and the other MISO data changes are updated when MISO_frame[DB14] bit2 is set
    MISO_frame[DB0] = 0x00;
    MISO_frame[DB1] = 0x00;
    MISO_frame[DB2] = 0x00;

    if (erropdata_cnt > 0) {  // error operating data available
      MISO_frame[DB6] = 0x80;
      MISO_frame[DB9] = 0xff;
      erropdata_cnt--;
    }

    // set Power, Mode, Tsetpoint, Fan, Vanes
    MISO_frame[DB0] = this->power_new;
    this->power_new = 0;

    MISO_frame[DB0] |= this->mode_new;
    this->mode_new = 0;

    MISO_frame[DB2] = this->tsetpoint_new;
    this->tsetpoint_new = 0;

    MISO_frame[DB1] = this->fan_new;
    this->fan_new = 0;

    MISO_frame[DB0] |= this->vanes_horizontal_0_new;
    MISO_frame[DB1] |= this->vanes_horizontal_1_new;
    this->vanes_horizontal_0_new = 0;
    this->vanes_horizontal_1_new = 0;

    MISO_frame[DB16] = 0;
    MISO_frame[DB16] |= this->new_vanes_vertical_1;
    MISO_frame[DB17] = 0;
    MISO_frame[DB17] |= this->new_vanes_vertical_0;
    MISO_frame[DB17] |= vanes_3dauto_new;
    vanes_3dauto_new = 0;
    this->new_vanes_vertical_0 = 0;
    this->new_vanes_vertical_1 = 0;

    if (request_erropData) {
      MISO_frame[DB6] = 0x80;
      MISO_frame[DB9] = 0x45;
      this->error_operating_data = false;
    }
  }

  MISO_frame[DB3] = this->troom_new;  // from MQTT or DS18x20

  uint16_t checksum = calc_checksum(MISO_frame, MHI_FRAME_SIZE_STANDARD);
  MISO_frame[CBH] = highByte(checksum);
  MISO_frame[CBL] = lowByte(checksum);

  checksum = calc_checksum(MISO_frame, MHI_FRAME_SIZE_EXTENDED);
  MISO_frame[CBL2] = lowByte(checksum);

  // read/write MOSI/MISO frame
  for (uint8_t byte_cnt = 0; byte_cnt < this->framesize; byte_cnt++) {  // read and write a data packet of 20 bytes
    MOSI_byte = 0;
    uint8_t bit_mask = 1;
    for (uint8_t bit_cnt = 0; bit_cnt < 8; bit_cnt++) {  // read and write 1 byte
      clock_ms = millis();

      while (digitalRead(SCK_PIN) != LOW) {  // wait for falling edge
        if (millis() - start_ms > max_time_ms)
          return ERR_MSG_TIMEOUT_CLOCK_HIGH;  // SCK stuck@ high error detection
      }

      if ((MISO_frame[byte_cnt] & bit_mask) == 0)
        digitalWrite(MISO_PIN, HIGH);
      else
        digitalWrite(MISO_PIN, LOW);

      while (!digitalRead(SCK_PIN) != HIGH) {  // wait for rising edge
        if ((millis() - startMillis) > max_time_ms)
          return err_msg_timeout_SCK_low;  // SCK stuck@ low error detection
      }

      if (digitalRead(MOSI_PIN) != HIGH)
        MOSI_byte += bit_mask;

      bit_mask = bit_mask << 1;
    }

    if (MOSI_frame[byte_cnt] != MOSI_byte) {
      new_datapacket_received = true;
      MOSI_frame[byte_cnt] = MOSI_byte;
    }
  }

  if (((MOSI_frame[SB0] & 0xfe) != 0x6c) | (MOSI_frame[SB1] != 0x80) | (MOSI_frame[SB2] != 0x04))
    return ERR_MSG_INVALID_SIGNATURE;

  checksum = calc_checksum(MOSI_frame, MHI_FRAME_SIZE_STANDARD);
  if (((MOSI_frame[CBH] << 8) | MOSI_frame[CBL]) != checksum)
    return ERR_MSG_INVALID_CHECKSUM;

  if (this->framesize == MHI_FRAME_SIZE_EXTENDED) {  // Only for framesize 33 (WF-RAC)
    checksum = calc_checksum(MOSI_frame, MHI_FRAME_SIZE_EXTENDED);
    if (MOSI_frame[CBL2] != lowByte(checksum))
      return ERR_MSG_INVALID_CHECKSUM;
  }

  if (new_datapacket_received) {
    if (this->framesize == MHI_FRAME_SIZE_EXTENDED) {  // Only for framesize 33 (WF-RAC)
      uint8_t vanes_vertical_tmp = (MOSI_frame[DB16] & 0x07) + ((MOSI_frame[DB17] & 0x01) << 4);
      if (vanes_vertical_tmp != this->status_vertical_old) {  // Vanes Left Right
        if ((vanes_vertical_tmp & 0x10) != 0)  // Vanes LR status swing
          this->status_cb->cbiStatusFunction(ACSTATUS_VANES_VERTICAL, ACVANES_VERTICAL_SWING);
        else
          this->status_cb->cbiStatusFunction(ACSTATUS_VANES_VERTICAL, (vanes_vertical_tmp & 0x07) + 1);
        this->status_vertical_old = vanes_vertical_tmp;
      }

      if ((MOSI_frame[DB17] & 0x04) != this->ACSTATUS_3DAUTO_old) {  // 3D auto
        this->acstatus_3dauto_old = MOSI_frame[DB17] & 0x04;
        this->status_cb->cbiStatusFunction(ACSTATUS_3DAUTO, this->acstatus_3dauto_old);
      }
    }
    // evaluate status
    if ((MOSI_frame[DB0] & 0x1c) != this->ACSTATUS_MODE_old) {  // Mode
      this->acstatus_mode_old = MOSI_frame[DB0] & 0x1c;
      this->status_cb->cbiStatusFunction(ACSTATUS_MODE, this->acstatus_mode_old);
    }

    if ((MOSI_frame[DB0] & 0x01) != this->acstatus_power_old) {  // Power
      this->acstatus_power_old = MOSI_frame[DB0] & 0x01;
      this->status_cb->cbiStatusFunction(ACSTATUS_POWER, this->acstatus_power_old);
    }

    uint8_t fantmp = MOSI_frame[DB1] & 0x07;
    if (fantmp != this->acstatus_fan_old) {
      this->acstatus_fan_old = fantmp;
      this->status_cb->cbiStatusFunction(ACSTATUS_FAN, this->acstatus_fan_old);
    }

    // Only updated when Vanes command via wired RC
    uint8_t vanes_horizontal_tmp = (MOSI_frame[DB0] & 0xc0) + ((MOSI_frame[DB1] & 0xB0) >> 4);
    if (vanes_horizontal_tmp != this->ACSTATUS_VANES_HORIZONTAL_old) {
      if ((vanes_horizontal_tmp & 0x88) == 0)  // last vanes update was via IR-RC, so status is not known
        this->status_cb->cbiStatusFunction(status_horizontal_vanes, ACVANES_HORIZONTAL_UNKNOWN);
      else if ((vanes_horizontal_tmp & 0x40) != 0)  // Vanes status swing
        this->status_cb->cbiStatusFunction(ACSTATUS_VANES_HORIZONTAL, ACVANES_HORIZONTAL_SWING);
      else
        this->status_cb->cbiStatusFunction(ACSTATUS_VANES_HORIZONTAL, (vanes_horizontal_tmp & 0x03) + 1);
      this->acstatus_vanes_horizontal_old = vanes_horizontal_tmp;
    }

    if(MOSI_frame[DB3] != this->acstatus_troom_old) {
      // To avoid jitter with the fast changing AC internal temperature sensor
      if (MISO_frame[DB3] != ROOM_TEMPERATURE_INTERNAL_SENSOR) {  // not internal sensor used, just publish
        this->acstatus_troom_old = MOSI_frame[DB3];
        this->status_cb->cbiStatusFunction(ACSTATUS_TROOM, this->acstatus_troom_old);
        last_troom_interval_ms = 0;
      }
      else  // internal sensor used
      {
        if ((millis() - last_troom_interval_ms) > min_time_interval_troom_ms) {  // Only publish when last change was more then min_time_interval_troom_ms ago
          last_troom_interval_ms = millis();
          this->acstatus_troom_old = MOSI_frame[DB3];
          this->status_cb->cbiStatusFunction(ACSTATUS_TROOM, this->acstatus_troom_old);
        }
      }
    }

    if (MOSI_frame[DB2] != this->acstatus_tsetpoint_old) {  // Temperature setpoint
      this->acstatus_tsetpoint_old = MOSI_frame[DB2];
      this->status_cb->cbiStatusFunction(ACSTATUS_TSETPOINT, this->acstatus_tsetpoint_old);
    }

    if (MOSI_frame[DB4] != this->acstatus_errorcode_old) {  // error code
      this->acstatus_errorcode_old = MOSI_frame[DB4];
      this->status_cb->cbiStatusFunction(ACSTATUS_ERRORCODE, this->acstatus_errorcode_old);
    }

    // Evaluate Operating Data and Error Operating Data
    bool MOSI_type_opdata = (MOSI_frame[DB10] & 0x30) == 0x10;

    switch (MOSI_frame[DB9]) {
      case 0x94:  // 0 energy-kwh n * 0.25 kWh used since power on
        if ((MOSI_frame[DB6] & 0x80) != 0) {  //
          if (MOSI_type_opdata) {
            if (((MOSI_frame[DB12] << 8) + (MOSI_frame[DB11])) != this->op_kwh) {
              this->op_kwh = (MOSI_frame[DB12] << 8) + (MOSI_frame[DB11]);
              this->status_cb->cbiStatusFunction(ACSTATUS_OPDATA_kWh, this->op_kwh);
            }
          }
          //else
          //  this->status_cb->cbiStatusFunction(ACSTATUS_ERR_OPDATA_UNKNOWN, op_unknown);  // noch nie gesehen, dass es auftaucht
        }
        break;
      case 0x02:
        if ((MOSI_frame[DB6] & 0x80) != 0) {  // 1 MODE
          if (MOSI_type_opdata) {
            if ((MOSI_frame[DB10] != this->op_mode)) {
              this->op_mode = MOSI_frame[DB10];
              this->status_cb->cbiStatusFunction(ACSTATUS_OPDATA_MODE, (this->op_mode & 0x0f) << 2);
            }
          }
          else
          {
            this->status_cb->cbiStatusFunction(ACSTATUS_ERR_OPDATA_MODE, (MOSI_frame[DB10] & 0x0f) << 2);
          }
        }
        break;
      case 0x05:
        if ((MOSI_frame[DB6] & 0x80) != 0) {  // 2 SET-TEMP
          if (MOSI_frame[DB10] == 0x13) {
            if (MOSI_frame[DB11] != this->op_indoor_temperature) {
              this->op_indoor_temperature = MOSI_frame[DB11];
              this->status_cb->cbiStatusFunction(ACSTATUS_OPDATA_INDOOR_TEMPERATURE, this->op_indoor_temperature);
            }
          }
          else if (MOSI_frame[DB10] == 0x33) {
            this->status_cb->cbiStatusFunction(ACSTATUS_ERR_OPDATA_INDOOR_TEMPERATURE, MOSI_frame[DB11]);
          }
        }
        break;
      case 0x81:                              // 5 THI-R1 or 6 THI-R2
        if ((MOSI_frame[DB6] & 0x80) != 0) {  // 5 THI-R1
          if ((MOSI_frame[DB10] & 0x30) == 0x20) {
            if (MOSI_frame[DB11] != this->op_indoor_temperature_ubend) {
              this->op_indoor_temperature_ubend = MOSI_frame[DB11];
              this->status_cb->cbiStatusFunction(ACSTATUS_OPDATA_INDOOR_TEMPERATURE_UBEND, this->op_indoor_temperature_ubend);
            }
          } else {
            this->status_cb->cbiStatusFunction(ACSTATUS_ERR_OPDATA_INDOOR_TEMPERATURE_UBEND, MOSI_frame[DB11]);
          }
        } else {                              // 6 THI-R2
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB11] != this->op_indoor_temperature_capillary) {
              this->op_indoor_temperature_capillary = MOSI_frame[DB11];
              this->status_cb->cbiStatusFunction(ACSTATUS_OPDATA_INDOOR_TEMPERATURE_CAPILLARY, this->op_indoor_temperature_capillary);
            }
          } else {
            this->status_cb->cbiStatusFunction(ACSTATUS_ERR_OPDATA_INDOOR_TEMPERATURE_CAPILLARY, MOSI_frame[DB11]);
          }
        }
        break;
      case 0x87:
        if ((MOSI_frame[DB6] & 0x80) != 0) {  // 7 THI-R3
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB11] != this->op_indoor_temperature_suction_header) {
              this->op_indoor_temperature_suction_header = MOSI_frame[DB11];
              this->status_cb->cbiStatusFunction(ACSTATUS_OPDATA_INDOOR_TEMPERATURE_SUCTION_HEADER, this->op_indoor_temperature_suction_header);
            }
          } else {
            this->status_cb->cbiStatusFunction(ACSTATUS_ERR_OPDATA_INDOOR_TEMPERATURE_SUCTION_HEADER, MOSI_frame[DB11]);
          }
        }
        break;
      case 0x80:                              // 3 RETURN-AIR or 21 OUTDOOR
        if ((MOSI_frame[DB6] & 0x80) != 0) {  // 3 RETURN-AIR
          if ((MOSI_frame[DB10] & 0x30) == 0x20) {  // operating Data
            if (MOSI_frame[DB11] != this->op_indoor_temperature_return_air) {
              this->op_indoor_temperature_return_air = MOSI_frame[DB11];
              this->status_cb->cbiStatusFunction(ACSTATUS_OPDATA_INDOOR_TEMPERATURE_RETURN_AIR, this->op_indoor_temperature_return_air);
            }
          } else {
            this->status_cb->cbiStatusFunction(ACSTATUS_ERR_OPDATA_INDOOR_TEMPERATURE_RETURN_AIR, MOSI_frame[DB11]);
          }
        } else {                              // 21 OUTDOOR
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB11] != this->op_outdoor_temperature) {
              this->op_outdoor_temperature = MOSI_frame[DB11];
              this->status_cb->cbiStatusFunction(ACSTATUS_OPDATA_OUTDOOR_TEMPERATURE, this->op_outdoor_temperature);
            }
          } else {
            this->status_cb->cbiStatusFunction(ACSTATUS_ERR_OPDATA_OUTDOOR_TEMPERATURE, MOSI_frame[DB11]);
          }
        }
        break;
      case 0x1f:                              // 8 IU-FANSPEED or 34 OU-FANSPEED
        if ((MOSI_frame[DB6] & 0x80) != 0) {  // 8 IU-FANSPEED
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB10] != this->op_indoor_fanspeed) {
              this->op_indoor_fanspeed = MOSI_frame[DB10];
              this->status_cb->cbiStatusFunction(ACSTATUS_OPDATA_INDOOR_FANSPEED, this->op_indoor_fanspeed & 0x0f);
            }
          } else {
            this->status_cb->cbiStatusFunction(ACSTATUS_ERR_OPDATA_INDOOR_FANSPEED, MOSI_frame[DB10] & 0x0f);
          }
        } else {                              // 34 OU-FANSPEED
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB10] != this->op_outdoor_fanspeed) {
              this->op_outdoor_fanspeed = MOSI_frame[DB10];
              this->status_cb->cbiStatusFunction(ACSTATUS_OPDATA_OUTDOOR_FANSPEED, this->op_outdoor_fanspeed & 0x0f);
            }
          } else {
            this->status_cb->cbiStatusFunction(ACSTATUS_ERR_OPDATA_OUTDOOR_FANSPEED, MOSI_frame[DB10] & 0x0f);
          }
        }
        break;
      case 0x1e:                              // 12 TOTAL-IU-RUN or 37 TOTAL-COMP-RUN
        if ((MOSI_frame[DB6] & 0x80) != 0) {  // 12 TOTAL-IU-RUN
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB11] != this->op_indoor_total_runtime) {
              this->op_indoor_total_runtime = MOSI_frame[DB11];
              this->status_cb->cbiStatusFunction(ACSTATUS_OPDATA_INDOOR_TOTAL_RUNTIME, this->op_indoor_total_runtime);
            }
          } else {
            this->status_cb->cbiStatusFunction(ACSTATUS_ERR_OPDATA_INDOOR_TOTAL_RUNTIME, MOSI_frame[DB11]);
          }
        } else {                              // 37 TOTAL-COMP-RUN
          if (MOSI_frame[DB10] == 0x11) {
            if (MOSI_frame[DB11] != this->op_outdoor_total_compressor_runtime) {
              this->op_outdoor_total_compressor_runtime = MOSI_frame[DB11];
              this->status_cb->cbiStatusFunction(ACSTATUS_OPDATA_TOTAL_COMPRESSOR_RUNTIME, this->op_outdoor_total_compressor_runtime);
            }
          } else {
            this->status_cb->cbiStatusFunction(ACSTATUS_ERR_OPDATA_TOTAL_COMPRESSOR_RUNTIME, MOSI_frame[DB11]);
          }
        }
        break;
      case 0x82:
        if ((MOSI_frame[DB6] & 0x80) == 0) {  // 22 ThO-R1
          if (MOSI_type_opdata) {  // operating data
            if (MOSI_frame[DB11] != this->op_outdoor_temperature_heat_exchanger) {
              this->op_outdoor_temperature_heat_exchanger = MOSI_frame[DB11];
              this->status_cb->cbiStatusFunction(ACSTATUS_OPDATA_OUTDOOR_TEMPERATURE_HEAT_EXCHANGER, this->op_outdoor_temperature_heat_exchanger);
            }
          } else {
            this->status_cb->cbiStatusFunction(ACSTATUS_ERR_OPDATA_OUTDOOR_TEMPERATURE_HEAT_EXCHANGER, MOSI_frame[DB11]);
          }
        }
        break;
      case 0x11:
        if ((MOSI_frame[DB6] & 0x80) == 0) {  // 24 COMP
          if (MOSI_type_opdata) {
            if (((MOSI_frame[DB10] << 8) | MOSI_frame[DB11]) != this->op_outdoor_compressor_frequency) {
              this->op_outdoor_compressor_frequency = (MOSI_frame[DB10] << 8) | MOSI_frame[DB11];
              this->status_cb->cbiStatusFunction(ACSTATUS_OPDATA_COMPRESSOR_FREQUENCY, this->op_outdoor_compressor_frequency);
            }
          } else {
            this->status_cb->cbiStatusFunction(ACSTATUS_ERR_OPDATA_COMPRESSOR_FREQUENCY, ((MOSI_frame[DB10] << 8) | MOSI_frame[DB11]));
          }
        }
        break;
      case 0x85:
        if ((MOSI_frame[DB6] & 0x80) == 0) {  // 27 Td
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB11] != this->op_outdoor_temperature_d) {
              this->op_outdoor_temperature_d = MOSI_frame[DB11];
              this->status_cb->cbiStatusFunction(ACSTATUS_OPDATA_OUTDOOR_TEMPERATURE_D, this->op_outdoor_temperature_d);
            }
          } else {
            this->status_cb->cbiStatusFunction(ACSTATUS_ERR_OPDATA_OUTDOOR_TEMPERATURE_D, MOSI_frame[DB11]);
          }
        }
        break;
      case 0x90:
        if ((MOSI_frame[DB6] & 0x80) == 0) {  // 29 CT
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB11] != this->op_outdoor_ct_current) {
              this->op_outdoor_ct_current = MOSI_frame[DB11];
              this->status_cb->cbiStatusFunction(ACSTATUS_OPDATA_CT_CURRENT, this->op_outdoor_ct_current);
            }
          } else {
            this->status_cb->cbiStatusFunction(ACSTATUS_ERR_OPDATA_CT_CURRENT, MOSI_frame[DB11]);
          }
        }
        break;
      case 0xb1:
        if ((MOSI_frame[DB6] & 0x80) == 0) {  // 32 TDSH
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB11] != this->op_outdoor_temperature_dsh) {
              this->op_outdoor_temperature_dsh = MOSI_frame[DB11];
              this->status_cb->cbiStatusFunction(ACSTATUS_OPDATA_OUTDOOR_TEMPERATURE_DSH, this->op_outdoor_temperature_dsh / 2);
            }
          }
        }
        break;
      case 0x7c:
        if ((MOSI_frame[DB6] & 0x80) == 0) {  // 33 PROTECTION-No
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB11] != this->op_outdoor_protection_no) {
              this->op_outdoor_protection_no = MOSI_frame[DB11];
              this->status_cb->cbiStatusFunction(ACSTATUS_OPDATA_PROTECTION_NO, this->op_outdoor_protection_no);
            }
          }
        }
        break;
      case 0x0c:
        if ((MOSI_frame[DB6] & 0x80) == 0) {  // 36 DEFROST
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB10] != this->op_outdoor_defrost) {
              this->op_outdoor_defrost = MOSI_frame[DB10];
              this->status_cb->cbiStatusFunction(ACSTATUS_OPDATA_DEFROST, this->op_outdoor_defrost & 0b1);
            }
          }
        }
        break;
      case 0x13:
        if ((MOSI_frame[DB6] & 0x80) == 0) {  // 38 OU-EEV
          if (MOSI_type_opdata) {
            if (((MOSI_frame[DB12] << 8) | MOSI_frame[DB11]) != this->op_outdoor_eev1) {
              this->op_outdoor_eev1 = (MOSI_frame[DB12] << 8) | MOSI_frame[DB11];
              this->status_cb->cbiStatusFunction(ACSTATUS_OPDATA_OUTDOOR_EEV1, this->op_outdoor_eev1);
            }
          } else {
            this->status_cb->cbiStatusFunction(ACSTATUS_ERR_OPDATA_OUTDOOR_EEV1, (MOSI_frame[DB12] << 8) | MOSI_frame[DB11]);
          }
        }
        break;
      case 0x45:  // last error number or count of following error operating data
        if ((MOSI_frame[DB6] & 0x80) != 0) {
          if (MOSI_frame[DB10] == 0x11) {  // last error number
            this->status_cb->cbiStatusFunction(ACSTATUS_ERR_OPDATA_ERRORCODE, MOSI_frame[DB11]);
          } else if (MOSI_frame[DB10] == 0x12) { // count of following error operating data
            erropdata_cnt = MOSI_frame[DB11] + 4;
          }
        }
        break;
      case 0x00:  // dummy
        break;
      case 0xff:  // default
        break;
      default:    // unknown operating data
        this->status_cb->cbiStatusFunction(ACSTATUS_OPDATA_UNKNOWN, (MOSI_frame[DB10] << 8) | MOSI_frame[DB9]);
        Serial.printf("Unknown operating data, MOSI_frame[DB9]=%i MOSI_frame[D10]=%i\n", MOSI_frame[DB9], MOSI_frame[DB10]);
    }
  }

  return call_counter;
}
