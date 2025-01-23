// MHI-AC-Ctrol-core
// implements the core functions (read & write SPI)

#include <Arduino.h>
#include <stdbool.h>
#include <stdint.h>

#include "MHI-AC-Ctrl-core.h"

uint16_t calc_checksum(byte *frame) {
  uint16_t checksum = 0;

  for (int i = 0; i < CBH; i++)
    checksum += frame[i];

  return checksum;
}

uint16_t calc_checksumFrame33(byte *frame) {
  uint16_t checksum = 0;

  for (int i = 0; i < CBL2; i++)
    checksum += frame[i];

  return checksum;
}

void MHI_AC_Ctrl_Core::reset_old_values() {  // used e.g. when MQTT connection to broker is lost, to re-output data
  // old status
  this->status_power_old = 0xff;
  this->status_mode_old = 0xff;
  this->status_fan_old = 0xff;
  this->status_vanes_old = 0xff;
  this->status_troom_old = 0xfe;
  this->status_tsetpoint_old = 0x00;
  this->status_errorcode_old = 0xff;
  this->status_vanesLR_old = 0xff;
  this->status_3Dauto_old = 0xff;

  // old operating data
  this->op_kwh_old = 0xffff;
  this->op_mode_old = 0xff;
  this->op_settemp_old = 0xff;
  this->op_return_air_old = 0xff;
  this->op_iu_fanspeed_old = 0xff;
  this->op_thi_r1_old = 0x00;
  this->op_thi_r2_old = 0x00;
  this->op_thi_r3_old = 0x00;
  this->op_total_iu_run_old = 0;
  this->op_outdoor_old = 0xff;
  this->op_tho_r1_old = 0x00;
  this->op_total_comp_run_old = 0;
  this->op_ct_old = 0xff;
  this->op_tdsh_old = 0xff;
  this->op_protection_no_old = 0xff;
  this->op_ou_fanspeed_old = 0xff;
  this->op_defrost_old = 0x00;
  this->op_comp_old = 0xffff;
  this->op_td_old  = 0x00;
  this->op_ou_eev1_old = 0xffff;
}

void MHI_AC_Ctrl_Core::init() {
  pinMode(SCK_PIN, INPUT);
  pinMode(MOSI_PIN, INPUT);
  pinMode(MISO_PIN, OUTPUT);
  MHI_AC_Ctrl_Core::reset_old_values();
}

void MHI_AC_Ctrl_Core::set_power(bool power) {
  this->new_Power = 0b10 | power;
}

void MHI_AC_Ctrl_Core::set_mode(ACMode mode) {
  this->new_Mode = 0b00100000 | mode;
}

void MHI_AC_Ctrl_Core::set_tsetpoint(uint tsetpoint) {
  this->new_Tsetpoint = 0b10000000 | tsetpoint;
}

void MHI_AC_Ctrl_Core::set_fan(uint fan) {
  this->new_Fan = 0b00001000 | fan;
}

void MHI_AC_Ctrl_Core::set_3Dauto(AC3Dauto Dauto) {
  new_3Dauto = 0b00001010 | Dauto;
}

void MHI_AC_Ctrl_Core::set_vanes(uint vanes) {
  if (vanes == vanes_swing) {
    this->new_Vanes0 = 0b11000000;  // enable swing
  }
  else {
    this->new_Vanes0 = 0b10000000;  // disable swing
    this->new_Vanes1 = 0b10000000 | ((vanes - 1) << 4);
  }
}

void MHI_AC_Ctrl_Core::set_vanesLR(uint vanesLR) {
  if (vanesLR == vanesLR_swing) {
    this->new_VanesLR0 = 0b00001011;  // enable swing
  }
  else {
    this->new_VanesLR0 = 0b00001010;  // disable swing
    this->new_VanesLR1 = 0b00010000 | (vanesLR - 1);
  }
}

void MHI_AC_Ctrl_Core::request_ErrOpData() {
  this->request_erropData = true;
}

void MHI_AC_Ctrl_Core::set_troom(byte troom) {
  this->new_Troom = troom;
}

float MHI_AC_Ctrl_Core::get_troom_offset() {
  return this->Troom_offset;
}

void MHI_AC_Ctrl_Core::set_troom_offset(float offset) {
  this->Troom_offset = offset;
}

void MHI_AC_Ctrl_Core::set_frame_size(byte framesize) {
  if (framesize == 20 || framesize == 33)
    this->frameSize = framesize;
}

int MHI_AC_Ctrl_Core::loop(uint max_time_ms) {
  const byte opdataCnt = sizeof(opdata) / sizeof(byte) / 2;
  static byte opdataNo = 0;              //
  long startMillis = millis();           // start time of this loop run
  byte MOSI_byte;                        // received MOSI byte
  bool new_datapacket_received = false;  // indicated that a new frame was received
  static byte erropdataCnt = 0;          // number of expected error operating data
  static bool doubleframe = false;
  static int frame = 1;
  static byte MOSI_frame[33];
  //                            sb0   sb1   sb2   db0   db1   db2   db3   db4   db5   db6   db7   db8   db9  db10  db11  db12  db13  db14  chkH  chkL  db15  db16  db17  db18  db19  db20  db21  db22  db23  db24  db25  db26  chk2L
  static byte MISO_frame[] = { 0xA9, 0x00, 0x07, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x22 };

  static uint call_counter = 0;                     // counts how often this loop was called
  static unsigned long lastTroomInternalMillis = 0; // remember when Troom internal has changed

  if (this->frameSize == 33)
    MISO_frame[0] = 0xAA;

  call_counter++;
  int SCKMillis = millis();           // time of last SCK low level
  while ((millis() - SCKMillis) < 5) {  // wait for 5ms stable high signal to detect a frame start
    if (!digitalRead(SCK_PIN))
      SCKMillis = millis();
    if ((millis() - startMillis) > max_time_ms)
      return err_msg_timeout_SCK_low;  // SCK stuck@ low error detection
  }

  // build the next MISO frame
  doubleframe = !doubleframe;           // toggle every frame
  MISO_frame[DB14] = doubleframe << 2;  // MISO_frame[DB14] bit2 toggles with every frame

  // Requesting all different opdata's is an opdata cycle. A cycle will take 20s.
  // With the current 20 different opdata's, every opdata request will take 1sec (interval).
  // If there are only 5 different opdata's defined, these 5 will be spread about the 20s cycle. The interval will increase.
  // requesting a new opdata will always start at a doubleframe start
  if ((frame > (NoFramesPerOpDataCycle / opdataCnt)) && doubleframe)  // interval for requesting new opdata depending on de number of opdata requests
    frame = 1;  // start requesting new OpData

  if (frame++ <= 2) {   // use opdata request only for 2 subsequent frames
    if (doubleframe) {  // start when MISO_frame[DB14] bit2 is set
      if (erropdataCnt == 0) {
        MISO_frame[DB6] = pgm_read_word(opdata + opdataNo);
        MISO_frame[DB9] = pgm_read_word(opdata + opdataNo) >> 8;
        opdataNo = (opdataNo + 1) % opdataCnt;
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

    if (erropdataCnt > 0) {  // error operating data available
      MISO_frame[DB6] = 0x80;
      MISO_frame[DB9] = 0xff;
      erropdataCnt--;
    }

    // set Power, Mode, Tsetpoint, Fan, Vanes
    MISO_frame[DB0] = this->new_Power;
    this->new_Power = 0;

    MISO_frame[DB0] |= this->new_Mode;
    this->new_Mode = 0;

    MISO_frame[DB2] = this->new_Tsetpoint;
    this->new_Tsetpoint = 0;

    MISO_frame[DB1] = this->new_Fan;
    this->new_Fan = 0;

    MISO_frame[DB0] |= this->new_Vanes0;
    MISO_frame[DB1] |= this->new_Vanes1;
    this->new_Vanes0 = 0;
    this->new_Vanes1 = 0;

    if (this->request_erropData) {
      MISO_frame[DB6] = 0x80;
      MISO_frame[DB9] = 0x45;
      this->request_erropData = false;
    }
  }

  MISO_frame[DB3] = this->new_Troom;  // from MQTT or DS18x20

  uint16_t checksum = calc_checksum(MISO_frame);
  MISO_frame[CBH] = highByte(checksum);
  MISO_frame[CBL] = lowByte(checksum);

  if (this->frameSize == 33) {  // Only for framesize 33 (WF-RAC)
    MISO_frame[DB16] = 0;
    MISO_frame[DB16] |= this->new_VanesLR1;
    MISO_frame[DB17] = 0;
    MISO_frame[DB17] |= this->new_VanesLR0;
    MISO_frame[DB17] |= new_3Dauto;
    new_3Dauto = 0;
    this->new_VanesLR0 = 0;
    this->new_VanesLR1 = 0;

    checksum = calc_checksumFrame33(MISO_frame);
    MISO_frame[CBL2] = lowByte(checksum);
  }

  // read/write MOSI/MISO frame
  for (uint8_t byte_cnt = 0; byte_cnt < this->frameSize; byte_cnt++) {  // read and write a data packet of 20 bytes
    MOSI_byte = 0;
    byte bit_mask = 1;
    for (uint8_t bit_cnt = 0; bit_cnt < 8; bit_cnt++) {  // read and write 1 byte
      SCKMillis = millis();

      while (digitalRead(SCK_PIN)) {  // wait for falling edge
        if (millis() - startMillis > max_time_ms)
          return err_msg_timeout_SCK_high;  // SCK stuck@ high error detection
      }
      if ((MISO_frame[byte_cnt] & bit_mask) > 0)
        digitalWrite(MISO_PIN, 1);
      else
        digitalWrite(MISO_PIN, 0);

      while (!digitalRead(SCK_PIN)) {}  // wait for rising edge

      if (digitalRead(MOSI_PIN))
        MOSI_byte += bit_mask;

      bit_mask = bit_mask << 1;
    }

    if (MOSI_frame[byte_cnt] != MOSI_byte) {
      new_datapacket_received = true;
      MOSI_frame[byte_cnt] = MOSI_byte;
    }
  }

  checksum = calc_checksum(MOSI_frame);
  if (((MOSI_frame[SB0] & 0xfe) != 0x6c) | (MOSI_frame[SB1] != 0x80) | (MOSI_frame[SB2] != 0x04))
    return err_msg_invalid_signature;

  if (((MOSI_frame[CBH] << 8) | MOSI_frame[CBL]) != checksum)
    return err_msg_invalid_checksum;

  if (this->frameSize == 33) {  // Only for framesize 33 (WF-RAC)
    checksum = calc_checksumFrame33(MOSI_frame);
    if (MOSI_frame[CBL2] != lowByte(checksum))
      return err_msg_invalid_checksum;
  }

  if (new_datapacket_received) {
    if (this->frameSize == 33) {  // Only for framesize 33 (WF-RAC)
      byte vanesLRtmp = (MOSI_frame[DB16] & 0x07) + ((MOSI_frame[DB17] & 0x01) << 4);
      if (vanesLRtmp != this->status_vanesLR_old) {  // Vanes Left Right
        if ((vanesLRtmp & 0x10) != 0)  // Vanes LR status swing
          this->m_cbiStatus->cbiStatusFunction(status_vanesLR, vanesLR_swing);
        else
          this->m_cbiStatus->cbiStatusFunction(status_vanesLR, (vanesLRtmp & 0x07) + 1);
        this->status_vanesLR_old = vanesLRtmp;
      }

      if ((MOSI_frame[DB17] & 0x04) != this->status_3Dauto_old) {  // 3D auto
        this->status_3Dauto_old = MOSI_frame[DB17] & 0x04;
        this->m_cbiStatus->cbiStatusFunction(status_3Dauto, this->status_3Dauto_old);
      }
    }
    // evaluate status
    if ((MOSI_frame[DB0] & 0x1c) != this->status_mode_old) {  // Mode
      this->status_mode_old = MOSI_frame[DB0] & 0x1c;
      this->m_cbiStatus->cbiStatusFunction(status_mode, this->status_mode_old);
    }

    if ((MOSI_frame[DB0] & 0x01) != this->status_power_old) {  // Power
      this->status_power_old = MOSI_frame[DB0] & 0x01;
      this->m_cbiStatus->cbiStatusFunction(status_power, this->status_power_old);
    }

    uint fantmp = MOSI_frame[DB1] & 0x07;
    if (fantmp != this->status_fan_old) {
      this->status_fan_old = fantmp;
      this->m_cbiStatus->cbiStatusFunction(status_fan, this->status_fan_old);
    }

    // Only updated when Vanes command via wired RC
    uint vanestmp = (MOSI_frame[DB0] & 0xc0) + ((MOSI_frame[DB1] & 0xB0) >> 4);
    if (vanestmp != this->status_vanes_old) {
      if ((vanestmp & 0x88) == 0)  // last vanes update was via IR-RC, so status is not known
        this->m_cbiStatus->cbiStatusFunction(status_vanes, vanes_unknown);
      else if ((vanestmp & 0x40) != 0)  // Vanes status swing
        this->m_cbiStatus->cbiStatusFunction(status_vanes, vanes_swing);
      else
        this->m_cbiStatus->cbiStatusFunction(status_vanes, (vanestmp & 0x03) + 1);
      this->status_vanes_old = vanestmp;
    }

    if(MOSI_frame[DB3] != this->status_troom_old) {
      // To avoid jitter with the fast changing AC internal temperature sensor
      if (MISO_frame[DB3] != 0xff) {  // not internal sensor used, just publish
        this->status_troom_old = MOSI_frame[DB3];
        this->m_cbiStatus->cbiStatusFunction(status_troom, this->status_troom_old);
        lastTroomInternalMillis = 0;
      }
      else  // internal sensor used
      {
        if ((unsigned long)(millis() - lastTroomInternalMillis) > minTimeInternalTroom) {  // Only publish when last change was more then minTimeInternalTroom ago
          lastTroomInternalMillis = millis();
          this->status_troom_old = MOSI_frame[DB3];
          this->m_cbiStatus->cbiStatusFunction(status_troom, this->status_troom_old);
        }
      }
    }

    if (MOSI_frame[DB2] != this->status_tsetpoint_old) {  // Temperature setpoint
      this->status_tsetpoint_old = MOSI_frame[DB2];
      this->m_cbiStatus->cbiStatusFunction(status_tsetpoint, this->status_tsetpoint_old);
    }

    if (MOSI_frame[DB4] != this->status_errorcode_old) {  // error code
      this->status_errorcode_old = MOSI_frame[DB4];
      this->m_cbiStatus->cbiStatusFunction(status_errorcode, this->status_errorcode_old);
    }

    // Evaluate Operating Data and Error Operating Data
    bool MOSI_type_opdata = (MOSI_frame[DB10] & 0x30) == 0x10;

    switch (MOSI_frame[DB9]) {
      case 0x94:  // 0 energy-kwh n * 0.25 kWh used since power on
        if ((MOSI_frame[DB6] & 0x80) != 0) {  //
          if (MOSI_type_opdata) {
            if (((MOSI_frame[DB12] << 8) + (MOSI_frame[DB11])) != this->op_kwh_old) {
              this->op_kwh_old = (MOSI_frame[DB12] << 8) + (MOSI_frame[DB11]);
              this->m_cbiStatus->cbiStatusFunction(opdata_kwh, this->op_kwh_old);
            }
          }
          //else
          //  this->m_cbiStatus->cbiStatusFunction(erropdata_unknown, op_unknown_old);  // noch nie gesehen, dass es auftaucht
        }
        break;
      case 0x02:
        if ((MOSI_frame[DB6] & 0x80) != 0) {  // 1 MODE
          if (MOSI_type_opdata) {
            if ((MOSI_frame[DB10] != this->op_mode_old)) {
              this->op_mode_old = MOSI_frame[DB10];
              this->m_cbiStatus->cbiStatusFunction(opdata_mode, (this->op_mode_old & 0x0f) << 2);
            }
          }
          else
          {
            this->m_cbiStatus->cbiStatusFunction(erropdata_mode, (MOSI_frame[DB10] & 0x0f) << 2);
          }
        }
        break;
      case 0x05:
        if ((MOSI_frame[DB6] & 0x80) != 0) {  // 2 SET-TEMP
          if (MOSI_frame[DB10] == 0x13) {
            if (MOSI_frame[DB11] != this->op_settemp_old) {
              this->op_settemp_old = MOSI_frame[DB11];
              this->m_cbiStatus->cbiStatusFunction(opdata_tsetpoint, this->op_settemp_old);
            }
          }
          else if (MOSI_frame[DB10] == 0x33) {
            this->m_cbiStatus->cbiStatusFunction(erropdata_tsetpoint, MOSI_frame[DB11]);
          }
        }
        break;
      case 0x81:                              // 5 THI-R1 or 6 THI-R2
        if ((MOSI_frame[DB6] & 0x80) != 0) {  // 5 THI-R1
          if ((MOSI_frame[DB10] & 0x30) == 0x20) {
            if (MOSI_frame[DB11] != this->op_thi_r1_old) {
              this->op_thi_r1_old = MOSI_frame[DB11];
              this->m_cbiStatus->cbiStatusFunction(opdata_thi_r1, this->op_thi_r1_old);
            }
          } else {
            this->m_cbiStatus->cbiStatusFunction(erropdata_thi_r1, MOSI_frame[DB11]);
          }
        } else {                              // 6 THI-R2
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB11] != this->op_thi_r2_old) {
              this->op_thi_r2_old = MOSI_frame[DB11];
              this->m_cbiStatus->cbiStatusFunction(opdata_thi_r2, this->op_thi_r2_old);
            }
          } else {
            this->m_cbiStatus->cbiStatusFunction(erropdata_thi_r2, MOSI_frame[DB11]);
          }
        }
        break;
      case 0x87:
        if ((MOSI_frame[DB6] & 0x80) != 0) {  // 7 THI-R3
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB11] != this->op_thi_r3_old) {
              this->op_thi_r3_old = MOSI_frame[DB11];
              this->m_cbiStatus->cbiStatusFunction(opdata_thi_r3, this->op_thi_r3_old);
            }
          } else {
            this->m_cbiStatus->cbiStatusFunction(erropdata_thi_r3, MOSI_frame[DB11]);
          }
        }
        break;
      case 0x80:                              // 3 RETURN-AIR or 21 OUTDOOR
        if ((MOSI_frame[DB6] & 0x80) != 0) {  // 3 RETURN-AIR
          if ((MOSI_frame[DB10] & 0x30) == 0x20) {  // operating Data
            if (MOSI_frame[DB11] != this->op_return_air_old) {
              this->op_return_air_old = MOSI_frame[DB11];
              this->m_cbiStatus->cbiStatusFunction(opdata_return_air, this->op_return_air_old);
            }
          } else {
            this->m_cbiStatus->cbiStatusFunction(erropdata_return_air, MOSI_frame[DB11]);
          }
        } else {                              // 21 OUTDOOR
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB11] != this->op_outdoor_old) {
              this->op_outdoor_old = MOSI_frame[DB11];
              this->m_cbiStatus->cbiStatusFunction(opdata_outdoor, this->op_outdoor_old);
            }
          } else {
            this->m_cbiStatus->cbiStatusFunction(erropdata_outdoor, MOSI_frame[DB11]);
          }
        }
        break;
      case 0x1f:                              // 8 IU-FANSPEED or 34 OU-FANSPEED
        if ((MOSI_frame[DB6] & 0x80) != 0) {  // 8 IU-FANSPEED
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB10] != this->op_iu_fanspeed_old) {
              this->op_iu_fanspeed_old = MOSI_frame[DB10];
              this->m_cbiStatus->cbiStatusFunction(opdata_iu_fanspeed, this->op_iu_fanspeed_old & 0x0f);
            }
          } else {
            this->m_cbiStatus->cbiStatusFunction(erropdata_iu_fanspeed, MOSI_frame[DB10] & 0x0f);
          }
        } else {                              // 34 OU-FANSPEED
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB10] != this->op_ou_fanspeed_old) {
              this->op_ou_fanspeed_old = MOSI_frame[DB10];
              this->m_cbiStatus->cbiStatusFunction(opdata_ou_fanspeed, this->op_ou_fanspeed_old & 0x0f);
            }
          } else {
            this->m_cbiStatus->cbiStatusFunction(erropdata_ou_fanspeed, MOSI_frame[DB10] & 0x0f);
          }
        }
        break;
      case 0x1e:                              // 12 TOTAL-IU-RUN or 37 TOTAL-COMP-RUN
        if ((MOSI_frame[DB6] & 0x80) != 0) {  // 12 TOTAL-IU-RUN
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB11] != this->op_total_iu_run_old) {
              this->op_total_iu_run_old = MOSI_frame[DB11];
              this->m_cbiStatus->cbiStatusFunction(opdata_total_iu_run, this->op_total_iu_run_old);
            }
          } else {
            this->m_cbiStatus->cbiStatusFunction(erropdata_total_iu_run, MOSI_frame[DB11]);
          }
        } else {                              // 37 TOTAL-COMP-RUN
          if (MOSI_frame[DB10] == 0x11) {
            if (MOSI_frame[DB11] != this->op_total_comp_run_old) {
              this->op_total_comp_run_old = MOSI_frame[DB11];
              this->m_cbiStatus->cbiStatusFunction(opdata_total_comp_run, this->op_total_comp_run_old);
            }
          } else {
            this->m_cbiStatus->cbiStatusFunction(erropdata_total_comp_run, MOSI_frame[DB11]);
          }
        }
        break;
      case 0x82:
        if ((MOSI_frame[DB6] & 0x80) == 0) {  // 22 ThO-R1
          if (MOSI_type_opdata) {  // operating data
            if (MOSI_frame[DB11] != this->op_tho_r1_old) {
              this->op_tho_r1_old = MOSI_frame[DB11];
              this->m_cbiStatus->cbiStatusFunction(opdata_tho_r1, this->op_tho_r1_old);
            }
          } else {
            this->m_cbiStatus->cbiStatusFunction(erropdata_tho_r1, MOSI_frame[DB11]);
          }
        }
        break;
      case 0x11:
        if ((MOSI_frame[DB6] & 0x80) == 0) {  // 24 COMP
          if (MOSI_type_opdata) {
            if (((MOSI_frame[DB10] << 8) | MOSI_frame[DB11]) != this->op_comp_old) {
              this->op_comp_old = (MOSI_frame[DB10] << 8) | MOSI_frame[DB11];
              this->m_cbiStatus->cbiStatusFunction(opdata_comp, this->op_comp_old & 0x0fff);
            }
          } else {
            this->m_cbiStatus->cbiStatusFunction(erropdata_comp, ((MOSI_frame[DB10] << 8) | MOSI_frame[DB11]) & 0x0fff);
          }
        }
        break;
      case 0x85:
        if ((MOSI_frame[DB6] & 0x80) == 0) {  // 27 Td
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB11] != this->op_td_old) {
              this->op_td_old = MOSI_frame[DB11];
              this->m_cbiStatus->cbiStatusFunction(opdata_td, this->op_td_old);
            }
          } else {
            this->m_cbiStatus->cbiStatusFunction(erropdata_td, MOSI_frame[DB11]);
          }
        }
        break;
      case 0x90:
        if ((MOSI_frame[DB6] & 0x80) == 0) {  // 29 CT
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB11] != this->op_ct_old) {
              this->op_ct_old = MOSI_frame[DB11];
              this->m_cbiStatus->cbiStatusFunction(opdata_ct, this->op_ct_old);
            }
          } else {
            this->m_cbiStatus->cbiStatusFunction(erropdata_ct, MOSI_frame[DB11]);
          }
        }
        break;
      case 0xb1:
        if ((MOSI_frame[DB6] & 0x80) == 0) {  // 32 TDSH
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB11] != this->op_tdsh_old) {
              this->op_tdsh_old = MOSI_frame[DB11];
              this->m_cbiStatus->cbiStatusFunction(opdata_tdsh, this->op_tdsh_old / 2);
            }
          }
        }
        break;
      case 0x7c:
        if ((MOSI_frame[DB6] & 0x80) == 0) {  // 33 PROTECTION-No
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB11] != this->op_protection_no_old) {
              this->op_protection_no_old = MOSI_frame[DB11];
              this->m_cbiStatus->cbiStatusFunction(opdata_protection_no, this->op_protection_no_old);
            }
          }
        }
        break;
      case 0x0c:
        if ((MOSI_frame[DB6] & 0x80) == 0) {  // 36 DEFROST
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB10] != this->op_defrost_old) {
              this->op_defrost_old = MOSI_frame[DB10];
              this->m_cbiStatus->cbiStatusFunction(opdata_defrost, this->op_defrost_old & 0b1);
            }
          }
        }
        break;
      case 0x13:
        if ((MOSI_frame[DB6] & 0x80) == 0) {  // 38 OU-EEV
          if (MOSI_type_opdata) {
            if (((MOSI_frame[DB12] << 8) | MOSI_frame[DB11]) != this->op_ou_eev1_old) {
              this->op_ou_eev1_old = (MOSI_frame[DB12] << 8) | MOSI_frame[DB11];
              this->m_cbiStatus->cbiStatusFunction(opdata_ou_eev1, this->op_ou_eev1_old);
            }
          } else {
            this->m_cbiStatus->cbiStatusFunction(erropdata_ou_eev1, (MOSI_frame[DB12] << 8) | MOSI_frame[DB11]);
          }
        }
        break;
      case 0x45:  // last error number or count of following error operating data
        if ((MOSI_frame[DB6] & 0x80) != 0) {
          if (MOSI_frame[DB10] == 0x11) {  // last error number
            this->m_cbiStatus->cbiStatusFunction(erropdata_errorcode, MOSI_frame[DB11]);
          } else if (MOSI_frame[DB10] == 0x12) { // count of following error operating data
            erropdataCnt = MOSI_frame[DB11] + 4;
          }
        }
        break;
      case 0x00:  // dummy
        break;
      case 0xff:  // default
        break;
      default:    // unknown operating data
        this->m_cbiStatus->cbiStatusFunction(opdata_unknown, (MOSI_frame[DB10] << 8) | MOSI_frame[DB9]);
        Serial.printf("Unknown operating data, MOSI_frame[DB9]=%i MOSI_frame[D10]=%i\n", MOSI_frame[DB9], MOSI_frame[DB10]);
    }
  }

  return call_counter;
}
