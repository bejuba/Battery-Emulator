#ifndef INVERTERS_H
#define INVERTERS_H

#include "InverterProtocol.h"

#include <stdint.h>

// Inverter contactor workaround modes
enum inverter_contactor_mode_enum {
  NoWorkaround = 0,        // Normal operation - follow inverter's open/close requests
  AlwaysClosed = 1,        // Keep contactors always closed
  LockAfterFirstClose = 2  // Wait for first close request, then ignore opens
};
extern InverterProtocol* inverter;

// Call to initialize the build-time selected inverter. Safe to call even though inverter was not selected.
bool setup_inverter();

extern bool user_selected_vcu_contactor_control;
extern uint16_t user_selected_vcu_opmode_can_id;      // 11-bit standard ID of the opmode frame
extern uint8_t user_selected_vcu_opmode_byte;         // Byte index (0-7) of the opmode value
extern uint16_t user_selected_vcu_opmode_timeout_ms;  // No valid frame within this time means opmode off
extern bool
    user_selected_vcu_opmode_fault_open_immediately;  // Open at once on real battery fault, else hold until low current
extern uint16_t user_selected_vcu_opmode_open_delay_ms;  // Delay between opmode off and contactors opening
// True if the ID is already used by the Leaf emulation or by other traffic on the inverter bus
bool vcu_opmode_can_id_collides(uint16_t id);
// Opmode state for the web UI. last_opmode is -1 until the first valid frame has arrived
int16_t vcu_opmode_last();
bool vcu_opmode_fresh();
const char* vcu_opmode_name(int16_t opmode);
extern uint16_t user_selected_pylon_send;
extern uint16_t user_selected_inverter_cells;
extern uint16_t user_selected_inverter_modules;
extern uint16_t user_selected_inverter_cells_per_module;
extern uint16_t user_selected_inverter_voltage_level;
extern uint16_t user_selected_inverter_ah_capacity;
extern uint16_t user_selected_inverter_battery_type;
extern uint16_t user_selected_inverter_sungrow_type;
extern uint16_t user_selected_inverter_pylon_type;
extern uint16_t user_selected_inverter_foxess_type;
extern uint16_t user_selected_inverter_foxess_subtype;
extern uint16_t user_selected_inverter_foxess_modules;
extern inverter_contactor_mode_enum user_selected_inverter_contactor_mode;
extern bool user_selected_pylon_30koffset;
extern bool user_selected_pylon_invert_byteorder;
extern bool user_selected_inverter_deye_workaround;
extern bool user_selected_inverter_long_CAN_timeout;
extern bool user_selected_primo_gen24;
extern bool inverter_low_pass_filter;
extern bool charge_taper_soc;
extern uint16_t charge_taper_band_pptt;
extern uint16_t charge_taper_floor_W;

// BYD-Modbus (Fronius GenericStorage) ControlData block, registers 400-408.
// Default watchdog period in seconds, used until the inverter tells us otherwise via register 402.
static const uint32_t MODBUS_INV_WATCHDOG_DEFAULT_S = 60;
// Live watchdog period. Restored from NVM at boot, re-persisted whenever register 402 brings a new value.
extern uint32_t inverter_modbus_watchdog_timeout_s;
// Set by the inverter driver when the value above changed and needs persisting. The driver does not
// touch NVM itself; the core loop drains this into store_settings_inverter_watchdog().
extern bool inverter_modbus_watchdog_changed;
// Should a non-zero RebootCommand in register 407 restart the emulator?
extern bool user_selected_accept_inverter_reboot;
// Inverter wall clock from registers 403-406, Unix epoch seconds. 0 = nothing received since boot.
extern uint64_t inverter_modbus_utc_epoch_s;
#endif
