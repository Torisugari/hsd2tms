/* 
 *    Copyright (C) 2015 Torisugari <torisugari@gmail.com>
 *
 *     Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 *     The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 *    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */
#include "hsd2tms.h"
#include <iostream>
#include <chrono>
namespace hsd2tms {
void createTiles(uint32_t aMaxZoomLevel, const HimawariStandardData& aData,
                 DataType aType,
                 uint32_t aBand0, uint32_t aBand1, uint32_t aBand2) {
  if (!aData.mBands[aBand0].hasData() &&
      !aData.mBands[aBand1].hasData() &&
      !aData.mBands[aBand2].hasData()) {
    return;
  }

  int32_t z = aMaxZoomLevel;
  uint32_t max = 1 << z;
  for (uint32_t x = 0; x < max; x++) {
    for (uint32_t y = 0; y < max; y++) {
      createTile(z, x, y, aData, aType, aBand0, aBand1, aBand2);
    }
  }

  for (z--; z >= 0; z--) {
    max = 1 << z;
    for (uint32_t x = 0; x < max; x++) {
      for (uint32_t y = 0; y < max; y++) {
        shrinkTile(z, x, y, aType, aBand0, aBand1, aBand2);
      }
    }
  }
}

void createTiles(uint32_t aMaxZoomLevel, const HimawariStandardData& aData,
                 DataType aType,
                 uint32_t aBand0, uint32_t aBand1) {
  if (!aData.mBands[aBand0].hasData() &&
      !aData.mBands[aBand1].hasData()) {
    return;
  }

  int32_t z = aMaxZoomLevel;
  uint32_t max = 1 << z;
  for (uint32_t x = 0; x < max; x++) {
    for (uint32_t y = 0; y < max; y++) {
      createTile(z, x, y, aData, aType, aBand0, aBand1);
    }
  }

  for (z--; z >= 0; z--) {
    max = 1 << z;
    for (uint32_t x = 0; x < max; x++) {
      for (uint32_t y = 0; y < max; y++) {
        shrinkTile(z, x, y, aType, aBand0);
      }
    }
  }
}

void createTiles(uint32_t aMaxZoomLevel, const HimawariStandardData& aData,
                 DataType aType,
                 uint32_t aBand) {
  if (!aData.mBands[aBand].hasData()) {
    return;
  }

  int32_t z = aMaxZoomLevel;
  uint32_t max = 1 << z;
  for (uint32_t x = 0; x < max; x++) {
    for (uint32_t y = 0; y < max; y++) {
      createTile(z, x, y, aData, aType, aBand);
    }
  }

  for (z--; z >= 0; z--) {
    max = 1 << z;
    for (uint32_t x = 0; x < max; x++) {
      for (uint32_t y = 0; y < max; y++) {
        shrinkTile(z, x, y, aType, aBand);
      }
    }
  }
}

void createJapanTiles(int32_t aMinZoomLevel, const HimawariStandardData& aData,
                      DataType aType,
                      uint32_t aBand0, uint32_t aBand1, uint32_t aBand2) {
  if (aMinZoomLevel > 7) {
    return;
  }

  if (!aData.mBands[aBand0].hasData() &&
      !aData.mBands[aBand1].hasData() &&
      !aData.mBands[aBand2].hasData()) {
    return;
  }

  // lv8 Etorofu: 8/233/91
  // lv8 Yonakuni: 8/215/110
  // lv8 Okinotori: 8/224/113
  // lv8 Minamitori: 8/237/110
  static const uint32_t kJapanLeft8 = 214;
  static const uint32_t kJapanRight8 = 237;
  static const uint32_t kJapanTop8 = 90;
  static const uint32_t kJapanBottom8 = 114;

  uint32_t left = kJapanLeft8;
  uint32_t right = kJapanRight8;
  uint32_t top = kJapanTop8;
  uint32_t bottom = kJapanBottom8;

  int32_t z = 8;
  for (uint32_t x = left; x <= right; x++) {
    for (uint32_t y = top; y < bottom; y++) {
      createTile(z, x, y, aData, aType, aBand0, aBand1, aBand2);
    }
  }

  for (z-- ; z >= aMinZoomLevel; z--) {
    left = left >> 1;
    right = right >> 1;
    top = top >> 1;
    bottom = bottom >> 1;

    for (uint32_t x = left; x <= right; x++) {
      for (uint32_t y = top; y <= bottom; y++) {
        shrinkTile(z, x, y, aType, aBand0, aBand1, aBand2);
      }
    }
  }
}

void createJapanTiles(int32_t aMinZoomLevel, const HimawariStandardData& aData,
                      DataType aType,
                      uint32_t aBand0, uint32_t aBand1) {
  if (aMinZoomLevel > 7) {
    return;
  }

  if (!aData.mBands[aBand0].hasData() &&
      !aData.mBands[aBand1].hasData()) {
    return;
  }

  // lv8 Etorofu: 8/233/91
  // lv8 Yonakuni: 8/215/110
  // lv8 Okinotori: 8/224/113
  // lv8 Minamitori: 8/237/110
  static const uint32_t kJapanLeft8 = 214;
  static const uint32_t kJapanRight8 = 237;
  static const uint32_t kJapanTop8 = 90;
  static const uint32_t kJapanBottom8 = 114;

  uint32_t left = kJapanLeft8;
  uint32_t right = kJapanRight8;
  uint32_t top = kJapanTop8;
  uint32_t bottom = kJapanBottom8;

  int32_t z = 8;
  for (uint32_t x = left; x <= right; x++) {
    for (uint32_t y = top; y < bottom; y++) {
      createTile(z, x, y, aData, aType, aBand0, aBand1);
    }
  }

  for (z-- ; z >= aMinZoomLevel; z--) {
    left = left >> 1;
    right = right >> 1;
    top = top >> 1;
    bottom = bottom >> 1;

    for (uint32_t x = left; x <= right; x++) {
      for (uint32_t y = top; y <= bottom; y++) {
        shrinkTile(z, x, y, aType, aBand0);
      }
    }
  }
}

void createJapanTiles(int32_t aMinZoomLevel, const HimawariStandardData& aData,
                      DataType aType,
                      uint32_t aBand) {
  if (aMinZoomLevel > 7) {
    return;
  }

  if (!aData.mBands[aBand].hasData()) {
    return;
  }

  // lv8 Etorofu: 8/233/91
  // lv8 Yonakuni: 8/215/110
  // lv8 Okinotori: 8/224/113
  // lv8 Minamitori: 8/237/110
  static const uint32_t kJapanLeft8 = 214;
  static const uint32_t kJapanRight8 = 237;
  static const uint32_t kJapanTop8 = 90;
  static const uint32_t kJapanBottom8 = 114;

  uint32_t left = kJapanLeft8;
  uint32_t right = kJapanRight8;
  uint32_t top = kJapanTop8;
  uint32_t bottom = kJapanBottom8;

  int32_t z = 8;
  for (uint32_t x = left; x <= right; x++) {
    for (uint32_t y = top; y < bottom; y++) {
      createTile(z, x, y, aData, aType, aBand);
    }
  }

  for (z-- ; z >= aMinZoomLevel; z--) {
    left = left >> 1;
    right = right >> 1;
    top = top >> 1;
    bottom = bottom >> 1;

    for (uint32_t x = left; x <= right; x++) {
      for (uint32_t y = top; y <= bottom; y++) {
        shrinkTile(z, x, y, aType, aBand);
      }
    }
  }
}

} // hsd2tms

int main (int argc, char* argv[]) {
  auto start = std::chrono::system_clock::now();
  hsd2tms::HimawariStandardData himawariData(argc - 1);
  for (int i = 1; i < argc; i++) {
    himawariData.append(argv[i]);
  }
  himawariData.sort();

  hsd2tms::colorchart();
  hsd2tms::createTiles(8, himawariData, hsd2tms::TypeRadiation, 0, 1, 2);
  hsd2tms::createTiles(8, himawariData, hsd2tms::TypeRadiation, 3, 4, 5);
  hsd2tms::createTiles(8, himawariData, hsd2tms::TypeTemperature, 14);
  hsd2tms::createTiles(8, himawariData, hsd2tms::TypeDust, 14, 15);

#if 0
  hsd2tms::createJapanTiles(0, himawariData, hsd2tms::TypeDust, 12, 13);
  hsd2tms::createJapanTiles(0, himawariData, hsd2tms::TypeRadiation, 1);
  hsd2tms::createJapanTiles(0, himawariData, hsd2tms::TypeTemperature, 14);

  hsd2tms::createTiles(6, himawariData, hsd2tms::TypeRadiation, 3, 4, 5);
  hsd2tms::createTiles(5, himawariData, hsd2tms::TypeTemperature, 14);

  hsd2tms::CloudTopAltitude cta;
  hsd2tms::createAltitudeFile(6, 58, 25, himawariData, cta);
  hsd2tms::createTile(6, 58, 25, himawariData,
                      hsd2tms::TypeRadiation, 0, 1, 2);
#endif

  auto end = std::chrono::system_clock::now();
  std::cout << "Duration: "
            << std::chrono::duration_cast
                 <std::chrono::microseconds>(end - start).count()
            << " micro seconds ("
            << (std::chrono::duration_cast
                  <std::chrono::seconds>(end - start).count() / 60)
            << " minutes)\n";
  return 0;
}

