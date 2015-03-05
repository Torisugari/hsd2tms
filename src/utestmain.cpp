// CppUTest
#include "CppUTest/CommandLineTestRunner.h"
#include "hsd2tms.h"
#include <stdio.h>
#include <string>

// <HimawariStandardData.h>
TEST_GROUP(HimawariStandardDataTest) {
  hsd2tms::HimawariStandardData himawariData;
  TEST_SETUP() {
    char leaf[] = "HS_H08_20150125_0230_B01_FLDK_R10_S0110.DAT";
    std::string path;
#if 0
    for (int i = 1; i <= 10; i++) {
      snprintf(leaf, sizeof(leaf),
               "HS_H08_20150125_0230_B01_FLDK_R10_S%02d10.DAT", i);
      path.assign(HIMAWARI_TEST_DATA_DIR_PATH);
      path.append(leaf);
      himawariData.append(path.c_str());
    }

    for (int i = 1; i <= 10; i++) {
      snprintf(leaf, sizeof(leaf),
               "HS_H08_20150125_0230_B02_FLDK_R10_S%02d10.DAT", i);
      path.assign(HIMAWARI_TEST_DATA_DIR_PATH);
      path.append(leaf);
      himawariData.append(path.c_str());
    }

    for (int i = 1; i <= 10; i++) {
      snprintf(leaf, sizeof(leaf),
               "HS_H08_20150125_0230_B03_FLDK_R05_S%02d10.DAT", i);
      path.assign(HIMAWARI_TEST_DATA_DIR_PATH);
      path.append(leaf);
      himawariData.append(path.c_str());
    }
#else
    himawariData.append(HIMAWARI_TEST_DATA_DIR_PATH
                        "HS_H08_20150125_0230_B01_JP02_R10_S0101.DAT");
    himawariData.append(HIMAWARI_TEST_DATA_DIR_PATH
                        "HS_H08_20150125_0230_B02_JP02_R10_S0101.DAT");
    himawariData.append(HIMAWARI_TEST_DATA_DIR_PATH
                        "HS_H08_20150125_0230_B03_JP02_R05_S0101.DAT");
    himawariData.append(HIMAWARI_TEST_DATA_DIR_PATH
                        "HS_H08_20150125_0230_B04_JP02_R10_S0101.DAT");
    for (int i = 5; i <= 16; i++) {
      snprintf(leaf, sizeof(leaf),
               "HS_H08_20150125_0230_B%02d_JP02_R20_S0101.DAT", i);
      path.assign(HIMAWARI_TEST_DATA_DIR_PATH);
      path.append(leaf);
      himawariData.append(path.c_str());
    }
#endif
    himawariData.sort();
  }
  TEST_TEARDOWN() {
  }
};

TEST(HimawariStandardDataTest, FileHeaderCheck) {
  for (const hsd2tms::HimawariStandardDataSegment& segment:
         himawariData.mSegments) {
    const hsd2tms::BasicInfoBlock& bi = segment.mBasicInfoBlock;
    BYTES_EQUAL(0x10,          bi.body.mFlag1);
    STRCMP_EQUAL("Himawari-8", bi.body.mSatelliteName)

    const hsd2tms::NavigationInfoBlock& ni = segment.mNavigationInfoBlock;
    DOUBLES_EQUAL(140.71480,    ni.body.mSSPLongitude,   0.01);
    DOUBLES_EQUAL( -0.00760634, ni.body.mSSPLatitude,    0.01);
    DOUBLES_EQUAL(140.30461,    ni.body.mNadirLongitude, 0.01);
    DOUBLES_EQUAL( -0.0311837,  ni.body.mNadirLatitude,  0.01);
    const hsd2tms::ProjectionInfoBlock& pi = segment.mProjectionInfoBlock;
    DOUBLES_EQUAL(140.7,        pi.body.mSubLon,         __FLT_EPSILON__);
  }
}

// <TileMapService.h>
TEST_GROUP(TileMapServiceTest) {
  hsd2tms::TileMapService tmsData;
  TEST_SETUP() {}
  TEST_TEARDOWN() {}
};

TEST(TileMapServiceTest, coordCheck) {
  tmsData.init(0, 0, 0);

  DOUBLES_EQUAL(0.,
                tmsData.longitude(0) + tmsData.longitude(255),
                __FLT_EPSILON__);

  DOUBLES_EQUAL(0.,
                tmsData.latitude(0) + tmsData.latitude(255),
                __FLT_EPSILON__);

  tmsData.init(1, 1, 0);

  DOUBLES_EQUAL(M_PI,
                tmsData.longitude(0) + tmsData.longitude(255),
                __FLT_EPSILON__);

  DOUBLES_EQUAL(1.48387, tmsData.latitude(0), 0.00001);
}

// <hsd2tms.h>
TEST_GROUP(hsd2tmsUtil) {
  TEST_SETUP() {}
  TEST_TEARDOWN() {}
};

TEST(hsd2tmsUtil, ThermographPalette) {
  hsd2tms::PNGPalette& thermo = hsd2tms::ThermographPalette::getInstance();

  BYTES_EQUAL(0xCC, thermo.mAlphaTable[0]);
  BYTES_EQUAL(0xCC, thermo.mAlphaTable[0xFF]);

  BYTES_EQUAL(0x00, thermo.mPaletteTable[0x00].red);
  BYTES_EQUAL(0x01, thermo.mPaletteTable[0x00].green);
  BYTES_EQUAL(0xFF, thermo.mPaletteTable[0x00].blue);

  BYTES_EQUAL(0x3F, thermo.mPaletteTable[0x7F].red);
  BYTES_EQUAL(0xFF, thermo.mPaletteTable[0x7F].green);
  BYTES_EQUAL(0x40, thermo.mPaletteTable[0x7F].blue);

  BYTES_EQUAL(0x40, thermo.mPaletteTable[0x80].red);
  BYTES_EQUAL(0xFF, thermo.mPaletteTable[0x80].green);
  BYTES_EQUAL(0x3F, thermo.mPaletteTable[0x80].blue);

  BYTES_EQUAL(0xFF, thermo.mPaletteTable[0xFF].red);
  BYTES_EQUAL(0x01, thermo.mPaletteTable[0xFF].green);
  BYTES_EQUAL(0x00, thermo.mPaletteTable[0xFF].blue);
}

int main (int argc, char* argv[]) {
  return RUN_ALL_TESTS(argc, argv);
}
