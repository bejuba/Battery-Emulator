#ifndef VCU_CAN_H
#define VCU_CAN_H

#include "CanInverterProtocol.h"

extern bool user_selected_vcu_contactor_control;

class VCUInverter : public CanInverterProtocol {
 public:
  const char* name() override { return Name; }
  void update_values();
  void transmit_can(unsigned long currentMillis);
  void map_can_frame_to_variable(CAN_frame rx_frame);
  static constexpr const char* Name = "VCU mode: Nissan LEAF battery";

  // The ZombieVerter commands the contactors via its opmode spot value, sent with a CAN mapping.
  bool controls_contactor() override { return user_selected_vcu_contactor_control; }
  bool allows_contactor_closing() override;
  bool aborts_closing_when_not_allowed() override { return user_selected_vcu_contactor_control; }

  /* Expected opmode frame, set up in the ZombieVerter web UI (CAN mapping, direction Send):
   *   CAN ID    0x3A0 (928), 11-bit standard (configurable)
   *   Position  0 (= byte 0, configurable), length 8 bit, gain 1, offset 0, unsigned
   *   Period    100 ms (the Zombie sends all its custom CAN maps every 100 ms)
   *   Value     the "opmode" spot value
   * Watchdog: no frame within the timeout (default 500 ms) counts as opmode Off.
   *
   * ZombieVerter opmode enum (Stm32-vcu include/param_prj.h, OPMODES). This is the only place the
   * mapping is defined: Precharge/Run/Charge allow the contactors, everything else is treated as Off.
   * Preheat (5), PchFail (3) and any value above 5 are deliberately not allowed to close contactors. */
  enum VcuOpmode : uint8_t {
    MOD_OFF = 0,
    MOD_RUN = 1,
    MOD_PRECHARGE = 2,
    MOD_PCHFAIL = 3,
    MOD_CHARGE = 4,
    MOD_PREHEAT = 5
  };
  static bool opmode_allows_closing(uint8_t opmode) {
    return opmode == MOD_PRECHARGE || opmode == MOD_RUN || opmode == MOD_CHARGE;
  }

 private:
  unsigned long previousMillis10ms = 0;
  unsigned long previousMillis100ms = 0;
  unsigned long previousMillis500ms = 0;
  uint16_t remining_gids = 281;
  uint8_t mprun10 = 0;
  uint8_t mprun100 = 0;
  uint8_t counter_55B = 0;
  CAN_frame LEAF_1DC = {.FD = false,
                        .ext_ID = false,
                        .DLC = 8,
                        .ID = 0x1DC,
                        .data = {0x6E, 0x0C, 0x2F, 0xFD, 0x0C, 0x00, 0x00, 0xD8}};
  CAN_frame LEAF_1DB = {.FD = false,
                        .ext_ID = false,
                        .DLC = 8,
                        .ID = 0x1DB,
                        .data = {0xFF, 0xC0, 0xB1, 0xAB, 0x08, 0x00, 0x02, 0xB5}};
  CAN_frame LEAF_55B = {.FD = false,
                        .ext_ID = false,
                        .DLC = 8,
                        .ID = 0x55B,
                        .data = {0x27, 0x00, 0xAA, 0x00, 0xE1, 0x00, 0x10, 0x79}};
  CAN_frame LEAF_5BC = {.FD = false,
                        .ext_ID = false,
                        .DLC = 8,
                        .ID = 0x5BC,
                        .data = {0x16, 0x00, 0x14, 0x50, 0xC8, 0x02, 0xA1, 0x68}};
  CAN_frame LEAF_59E = {.FD = false,
                        .ext_ID = false,
                        .DLC = 8,
                        .ID = 0x59E,
                        .data = {0x00, 0x00, 0x0E, 0x60, 0x88, 0x00, 0x00, 0x00}};
  CAN_frame LEAF_5C0 = {.FD = false,
                        .ext_ID = false,
                        .DLC = 8,
                        .ID = 0x5C0,
                        .data = {0x80, 0x60, 0x60, 0x00, 0xC7, 0xB4, 0x04, 0x0C}};
};

#endif
