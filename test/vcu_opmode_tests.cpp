#include <gtest/gtest.h>

#include <Arduino.h>  // Emul: set_millis64() to control the test clock

#include "../Software/src/datalayer/datalayer.h"
#include "../Software/src/inverter/INVERTERS.h"
#include "../Software/src/inverter/VCU-CAN.h"

// Covers the VCU opmode contactor permission: boot default, healthy gating, watchdog,
// the open delay, immediate open on a real battery fault and the reserved-ID check.

namespace {

CAN_frame opmode_frame(uint8_t opmode) {
  CAN_frame f = {.FD = false, .ext_ID = false, .DLC = 8, .ID = 0x3A0, .data = {0}};
  f.data.u8[user_selected_vcu_opmode_byte] = opmode;
  return f;
}

}  // namespace

class VcuOpmodeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    datalayer = DataLayer();
    set_millis64(100000);
    user_selected_vcu_contactor_control = true;
    user_selected_vcu_opmode_can_id = 0x3A0;
    user_selected_vcu_opmode_byte = 0;
    user_selected_vcu_opmode_timeout_ms = 500;
    user_selected_vcu_opmode_open_delay_ms = 500;
    datalayer.battery.status.real_bms_status = BMS_ACTIVE;
    datalayer.system.status.system_status = ACTIVE;
    // Settle the file-static permission latch from earlier tests
    set_millis64(200000);
    vcu.map_can_frame_to_variable(opmode_frame(VCUInverter::MOD_OFF));
    vcu.allows_contactor_closing();
    set_millis64(201000);
    vcu.allows_contactor_closing();
  }

  void TearDown() override { user_selected_vcu_contactor_control = false; }

  void at(unsigned long ms) { set_millis64(ms); }

  VCUInverter vcu;
};

TEST_F(VcuOpmodeTest, AllowedInRunAndRevokedAfterDelayWhenOff) {
  at(300000);
  vcu.map_can_frame_to_variable(opmode_frame(VCUInverter::MOD_RUN));
  EXPECT_TRUE(vcu.allows_contactor_closing());

  at(300100);
  vcu.map_can_frame_to_variable(opmode_frame(VCUInverter::MOD_OFF));
  EXPECT_TRUE(vcu.allows_contactor_closing());  // Open delay running
  at(300400);
  EXPECT_TRUE(vcu.allows_contactor_closing());
  at(300700);
  EXPECT_FALSE(vcu.allows_contactor_closing());
}

TEST_F(VcuOpmodeTest, UnknownAndPreheatValuesAreOff) {
  at(300000);
  for (uint8_t value : std::initializer_list<uint8_t>{VCUInverter::MOD_PCHFAIL, VCUInverter::MOD_PREHEAT, 0x42}) {
    vcu.map_can_frame_to_variable(opmode_frame(value));
    EXPECT_FALSE(vcu.allows_contactor_closing());
  }
}

TEST_F(VcuOpmodeTest, WatchdogTimeoutMeansOff) {
  at(300000);
  vcu.map_can_frame_to_variable(opmode_frame(VCUInverter::MOD_CHARGE));
  EXPECT_TRUE(vcu.allows_contactor_closing());
  at(300499);
  EXPECT_TRUE(vcu.allows_contactor_closing());
  at(300500);  // Stale: delay starts here
  EXPECT_TRUE(vcu.allows_contactor_closing());
  at(301000);
  EXPECT_FALSE(vcu.allows_contactor_closing());
}

TEST_F(VcuOpmodeTest, NotGrantedWithoutHealthyBattery) {
  datalayer.battery.status.real_bms_status = BMS_DISCONNECTED;
  at(300000);
  vcu.map_can_frame_to_variable(opmode_frame(VCUInverter::MOD_PRECHARGE));
  EXPECT_FALSE(vcu.allows_contactor_closing());
}

TEST_F(VcuOpmodeTest, RealBatteryFaultOpensImmediately) {
  at(300000);
  vcu.map_can_frame_to_variable(opmode_frame(VCUInverter::MOD_RUN));
  EXPECT_TRUE(vcu.allows_contactor_closing());
  datalayer.battery.status.real_bms_status = BMS_FAULT;
  at(300010);
  EXPECT_FALSE(vcu.allows_contactor_closing());
}

TEST_F(VcuOpmodeTest, ConfiguredByteIsUsed) {
  user_selected_vcu_opmode_byte = 3;
  at(300000);
  vcu.map_can_frame_to_variable(opmode_frame(VCUInverter::MOD_RUN));
  EXPECT_TRUE(vcu.allows_contactor_closing());
  EXPECT_EQ(vcu_opmode_last(), VCUInverter::MOD_RUN);
}

TEST(VcuOpmodeIdTest, DefaultIsFreeAndLeafFramesCollide) {
  EXPECT_FALSE(vcu_opmode_can_id_collides(0x3A0));
  EXPECT_TRUE(vcu_opmode_can_id_collides(0x1DB));
  EXPECT_TRUE(vcu_opmode_can_id_collides(0x1F2));
  EXPECT_TRUE(vcu_opmode_can_id_collides(0x79B));
}
