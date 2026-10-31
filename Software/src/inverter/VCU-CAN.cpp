#include <Arduino.h>

#include "../communication/can/comm_can.h"
#include "../datalayer/datalayer.h"
#include "../devboard/utils/common_functions.h"  //For CRC table
#include "../inverter/INVERTERS.h"
#include "VCU-CAN.h"

/*TODO once testing starts:
- Map more optional content into all message
- Add OBD2 responses for 79B /7BB polls
- Populate onboard-charger power limit (0x1DC byte2 low nibble + byte3 top 6 bits, "chargelimit"
in leafbms.cpp). This currently stays at 0 and needs a real value once we know what the VCU
expects here, since leafbms.cpp maps it (in Amps) straight into Param::BMS_ChargeLim.
-*/

static uint8_t calculate_CRC_Nissan(CAN_frame* frame) {
  uint8_t crc = 0;
  for (uint8_t j = 0; j < 7; j++) {
    crc = crctable_nissan_leaf[(crc ^ static_cast<uint8_t>(frame->data.u8[j])) % 256];
  }
  return crc;
}

void VCUInverter::update_values() {  //Called every 1s

  //Fake that we have inverter alive, to avoid fault event (-1 to not trigger detected successfully message)
  datalayer.system.status.CAN_inverter_still_alive = (CAN_STILL_ALIVE - 1);

  // ---- 0x1DC: Discharge / charge power limits ----
  // leafbms.cpp decodes these as 10-bit fields in units of 0.25kW:
  // dislimit = ((byte0<<2) + (byte1>>6)) * 0.25
  // chglimit = (((byte1&0x3F)<<4) + (byte2>>4)) * 0.25
  // so the raw value = power_in_watts / 250.
  uint16_t dislimit_raw = static_cast<uint16_t>(datalayer.aggregate.max_discharge_power_W / 250) & 0x3FF;
  uint16_t chglimit_raw = static_cast<uint16_t>(datalayer.aggregate.max_charge_power_W / 250) & 0x3FF;

  LEAF_1DC.data.u8[0] = (dislimit_raw >> 2) & 0xFF;
  LEAF_1DC.data.u8[1] = static_cast<uint8_t>(((dislimit_raw & 0x03) << 6) | ((chglimit_raw >> 4) & 0x3F));
  LEAF_1DC.data.u8[2] = static_cast<uint8_t>((chglimit_raw & 0x0F) << 4);
  //Note: low nibble of byte2 + top 6 bits of byte3 are the onboard-charger power limit field
  //("chargelimit" in leafbms.cpp -> Param::BMS_ChargeLim). Left at 0 for now, see TODO above.

  // ---- 0x55B: State of charge ----
  // leafbms.cpp decodes: soc = ((byte0<<2) + (byte1>>6)) * 0.1
  // real_soc is stored as integer-percent x100 (9550 = 95.50%), so /10 gives 0.1%-per-bit units.
  uint16_t soc_raw = static_cast<uint16_t>(datalayer.aggregate.real_soc / 10) & 0x3FF;

  LEAF_55B.data.u8[0] = (soc_raw >> 2) & 0xFF;
  LEAF_55B.data.u8[1] = static_cast<uint8_t>((soc_raw & 0x03) << 6);

  // ---- 0x1DB: Pack current & voltage ----
  // leafbms.cpp decodes:
  // cur = (byte0<<3) + (byte1>>5); if (cur>1023) cur -= 2047; BattCur = cur/2
  // udc = (byte2<<2) + (byte3>>6); BattVoltage = udc/2
  // i.e. raw units are 0.5A/bit and 0.5V/bit, with current using an 11-bit field
  // biased by 2047 (not 2048) for negative values - this matches genuine Nissan LEAF traffic.
  int32_t current_raw = (static_cast<int32_t>(datalayer.aggregate.current_dA) * 2) / 10;
  if (current_raw < 0) {
    current_raw += 2047;
  }
  current_raw &= 0x7FF;

  uint16_t voltage_raw =
      static_cast<uint16_t>((static_cast<uint32_t>(datalayer.aggregate.voltage_dV) * 2) / 10) & 0x3FF;

  LEAF_1DB.data.u8[0] = (current_raw >> 3) & 0xFF;
  LEAF_1DB.data.u8[1] = static_cast<uint8_t>((current_raw & 0x07) << 5);
  LEAF_1DB.data.u8[2] = (voltage_raw >> 2) & 0xFF;
  LEAF_1DB.data.u8[3] = static_cast<uint8_t>(((voltage_raw & 0x03) << 6) | 0x2B);  //Lower 6 bits: status flags

  // ---- 0x5BC: Remaining GIDs & SOH ----
  // Not currently consumed by leafbms.cpp (SOH/GIDs decoding is commented out there),
  // but fixed here for correctness / future compatibility.
  uint16_t remaining_gids = static_cast<uint16_t>((datalayer.aggregate.real_soc / 10000.0) * 281);  //0-281 for 24kWh
  LEAF_5BC.data.u8[0] = (remaining_gids >> 2) & 0xFF;
  LEAF_5BC.data.u8[1] = static_cast<uint8_t>((remaining_gids & 0x03) << 6);
  LEAF_5BC.data.u8[4] = static_cast<uint8_t>((datalayer.aggregate.soh_pptt / 100) << 1);
}

namespace {
constexpr unsigned long VCU_OPEN_HOLD_MAX_MS = 10000;  // Max time to wait for the current to drop before opening
constexpr int32_t VCU_OPEN_CURRENT_THRESHOLD_dA = 18;  // 1.8 A, same as the equipment stop pause
// Opmode state, written by the CAN receive path and read by the contactor handler and web UI
int16_t last_opmode = -1;  // -1 until the first valid frame has arrived
unsigned long last_opmode_ms = 0;
bool permission = false;  // Latched permission handed to the contactor state machine
bool delay_running = false;
unsigned long delay_start_ms = 0;

/* IDs already used on the inverter bus: what the Leaf emulation sends and receives, the Leaf OBD2
 * request/response IDs and the Nissan LEAF charger frames. The opmode ID must not match any of them.
 * Note: other batteries or devices sharing this bus are not known here, e.g. the BMW batteries send 0x3A0. */
constexpr uint16_t RESERVED_IDS[] = {0x1DC, 0x1DB, 0x55B, 0x5BC, 0x59E, 0x5C0,  // Sent by the emulation
                                     0x1F2, 0x1D4, 0x50B, 0x50C,                // Received by the emulation
                                     0x79B, 0x7BB,                              // LEAF OBD2 request / response
                                     0x380, 0x5EB, 0x5BF, 0x1ED, 0x1C2,         // ZE1 battery frames
                                     0x679, 0x390, 0x393};                      // Nissan LEAF charger

bool opmode_fresh(unsigned long now) {
  return last_opmode >= 0 && (now - last_opmode_ms) < user_selected_vcu_opmode_timeout_ms;
}
}  // namespace

bool vcu_opmode_can_id_collides(uint16_t id) {
  for (uint16_t reserved : RESERVED_IDS) {
    if (reserved == id) {
      return true;
    }
  }
  return false;
}

int16_t vcu_opmode_last() {
  return last_opmode;
}

bool vcu_opmode_fresh() {
  return opmode_fresh(millis());
}

const char* vcu_opmode_name(int16_t opmode) {
  switch (opmode) {
    case VCUInverter::MOD_OFF:
      return "Off";
    case VCUInverter::MOD_RUN:
      return "Run";
    case VCUInverter::MOD_PRECHARGE:
      return "Precharge";
    case VCUInverter::MOD_PCHFAIL:
      return "PchFail";
    case VCUInverter::MOD_CHARGE:
      return "Charge";
    case VCUInverter::MOD_PREHEAT:
      return "Preheat";
    case -1:
      return "none received";
    default:
      return "unknown";
  }
}

/* Permission for the contactor state machine, evaluated every contactor handler pass.
 * - Granted only while opmode is Precharge/Run/Charge, the frame is fresh and the real battery is healthy.
 * - Revoked (opmode off/unknown, stale frame, or real battery fault): never remove torque abruptly. The
 *   permission is held until the pack current is near zero (below 1.8 A, as the equipment stop does) and at
 *   least the open delay has passed. The hold is bounded by VCU_OPEN_HOLD_MAX_MS, after which we open anyway.
 * - Real battery fault with "open immediately on fault" set: revoked at once.
 * Boot state is off until a valid frame arrives. */
bool VCUInverter::allows_contactor_closing() {
  unsigned long now = millis();
  bool bms_fault = datalayer.battery.status.real_bms_status == BMS_FAULT;

  if (bms_fault && user_selected_vcu_opmode_fault_open_immediately) {
    permission = false;
    delay_running = false;
    return false;
  }

  bool wanted = opmode_fresh(now) && opmode_allows_closing(static_cast<uint8_t>(last_opmode)) && !bms_fault;
  bool healthy =
      datalayer.system.status.system_status != FAULT && (datalayer.battery.status.real_bms_status == BMS_STANDBY ||
                                                         datalayer.battery.status.real_bms_status == BMS_ACTIVE);

  if (wanted) {
    delay_running = false;
    if (healthy) {
      permission = true;
    } else if (!permission) {
      return false;  // Not healthy, so do not start closing. If already closed, keep it, the fault paths handle it
    }
  } else if (permission) {
    if (!delay_running) {
      delay_running = true;
      delay_start_ms = now;
    }
    unsigned long held = now - delay_start_ms;
    bool current_low = abs(datalayer.aggregate.current_dA) < VCU_OPEN_CURRENT_THRESHOLD_dA;
    if (held >= VCU_OPEN_HOLD_MAX_MS) {
      logging.printf("VCU opmode: current still flowing after %lu ms, opening anyway\n", held);
      permission = false;
      delay_running = false;
    } else if (held >= user_selected_vcu_opmode_open_delay_ms && current_low) {
      permission = false;
      delay_running = false;
    }
  }
  return permission;
}

void VCUInverter::map_can_frame_to_variable(CAN_frame rx_frame) {
  if (user_selected_vcu_contactor_control && !rx_frame.ext_ID && rx_frame.ID == user_selected_vcu_opmode_can_id) {
    if (rx_frame.DLC > user_selected_vcu_opmode_byte) {
      int16_t opmode = rx_frame.data.u8[user_selected_vcu_opmode_byte];
      if (opmode != last_opmode) {
        logging.printf("VCU opmode: %s (%d)\n", vcu_opmode_name(opmode), opmode);
      }
      last_opmode = opmode;
      last_opmode_ms = millis();
      datalayer.system.status.CAN_inverter_still_alive = CAN_STILL_ALIVE;
    }
    return;
  }
  switch (rx_frame.ID) {
    case 0x1F2:
    case 0x1D4:
    case 0x50B:
    case 0x50C:
      datalayer.system.status.CAN_inverter_still_alive = CAN_STILL_ALIVE;
      break;
    default:
      break;
  }
}

void VCUInverter::transmit_can(unsigned long currentMillis) {
  // Send 10ms CAN Message
  if (currentMillis - previousMillis10ms >= INTERVAL_10_MS) {
    previousMillis10ms = currentMillis;

    mprun10 = (mprun10 + 1) % 4;  // mprun10 cycles between 0-1-2-3-0-1...
    LEAF_1DC.data.u8[6] = mprun10;
    LEAF_1DC.data.u8[7] = calculate_CRC_Nissan(&LEAF_1DC);
    transmit_can_frame(&LEAF_1DC);
    LEAF_1DB.data.u8[6] = mprun10;
    LEAF_1DB.data.u8[7] = calculate_CRC_Nissan(&LEAF_1DB);
    transmit_can_frame(&LEAF_1DB);
  }
  // Send 100ms CAN Messages
  if (currentMillis - previousMillis100ms >= INTERVAL_100_MS) {
    previousMillis100ms = currentMillis;

    mprun100 = (mprun100 + 1) % 4;  // mprun10 cycles between 0-1-2-3-0-1...
    counter_55B = (counter_55B + 1) % 10;
    if (counter_55B < 5) {
      LEAF_55B.data.u8[2] = 0xAA;
    } else {
      LEAF_55B.data.u8[2] = 0x55;
    }
    LEAF_55B.data.u8[6] = (1 << 4) | mprun100;  //1 = RefuseToSleep 2 = ReadyToSleep
    LEAF_55B.data.u8[7] = calculate_CRC_Nissan(&LEAF_55B);
    transmit_can_frame(&LEAF_55B);
    transmit_can_frame(&LEAF_5BC);
  }
  // Send 100ms CAN Messages
  if (currentMillis - previousMillis500ms >= INTERVAL_500_MS) {
    previousMillis500ms = currentMillis;

    transmit_can_frame(&LEAF_59E);
    if (LEAF_5C0.data.u8[0] == 0x40) {
      LEAF_5C0.data.u8[0] = 0x80;
    } else if (LEAF_5C0.data.u8[0] == 0x80) {
      LEAF_5C0.data.u8[0] = 0xC0;
    } else if (LEAF_5C0.data.u8[0] == 0xC0) {
      LEAF_5C0.data.u8[0] = 0x40;
    }
    transmit_can_frame(&LEAF_5C0);
  }
}
