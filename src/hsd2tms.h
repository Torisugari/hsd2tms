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
#ifndef HSD_TO_TMS_UTIL
#define HSD_TO_TMS_UTIL

#include <sstream>
#include <iostream>
#include <iomanip>

#include <cstring>
#include "png.h"

#include "HimawariStandardData.h"
#include "TileMapService.h"
#include "CloudTopAltitude.h"

namespace hsd2tms {

enum DataType {
  TypeRadiation,
  TypeTemperature,
  TypeDust
};

class DirNameProvider {
private:
  static void print0d(std::string& aStr, uint32_t aInt, uint32_t aOrder) {
    std::ostringstream stream;
    stream << std::setfill('0') << std::setw(aOrder) << aInt;
    aStr += stream.str();
  }
  inline static const char* typeTag(DataType aType) {
    static const char rad[] = "rad";
    static const char tem[] = "tem";
    static const char dst[] = "dst";
    static const char unknown[] = "unknown";

    switch(aType) {
      case TypeRadiation:
        return rad;
        break;
      case TypeTemperature:
        return tem;
      case TypeDust:
        return dst;
        break;
    }
    return unknown;
  }
public:
  static void compose(std::string& aPath, DataType aType,
                      uint32_t aBand) {
    aPath.assign("./");
    aPath.append(typeTag(aType));
    print0d(aPath, aBand + 1, 2);
    aPath += "/";
  }

  static void compose(std::string& aPath, DataType aType,
                      uint32_t aBand0, uint32_t aBand1) {
    aPath.assign("./");
    aPath.append(typeTag(aType));
    print0d(aPath, aBand0 + 1, 2);
    print0d(aPath, aBand1 + 1, 2);
    aPath += "/";
  }

  static void compose(std::string& aPath, DataType aType,
                      uint32_t aBand0, uint32_t aBand1, uint32_t aBand2) {
    aPath.assign("./");
    aPath.append(typeTag(aType));
    print0d(aPath, aBand0 + 1, 2);
    print0d(aPath, aBand1 + 1, 2);
    print0d(aPath, aBand2 + 1, 2);
    aPath += "/";
  }
};

struct PNGPalette {
  png_color mPaletteTable[256];
  uint8_t mAlphaTable[256];
  PNGPalette& operator= (const PNGPalette& aSource) {
    std::memcpy(mPaletteTable, aSource.mPaletteTable, sizeof(mPaletteTable));
    std::memcpy(mAlphaTable, aSource.mAlphaTable, sizeof(mAlphaTable));
    return (*this);
  }
};

// Helper class to fill mPaletteTable / mAlphaTable effectively.
class SingleColorPalette: public PNGPalette {
private:
  void Init(const png_color& aColor) {
    std::fill(std::begin(mPaletteTable), std::end(mPaletteTable), aColor);
    // Simple gradation with [alpha = index].
    for (int i = 0; i < 256; i++) {
      mAlphaTable[i] = i;
    }
  }
public:
  SingleColorPalette() {
    png_color color = {0, 0, 0};
    Init(color);
  }
  SingleColorPalette(uint8_t aRed, uint8_t aGreen, uint8_t aBlue) {
    png_color color = {aRed, aGreen, aBlue};
    Init(color);
  }
  SingleColorPalette(const png_color& aColor) {
    Init(aColor);
  }
};

// Palette a la thermography.
class ThermographPalette: public PNGPalette {
  ThermographPalette() {
    std::fill(std::begin(mAlphaTable), std::end(mAlphaTable), 0xCC);
    for (int i = 0; i < 256; i++) {
      mPaletteTable[i].red = (i * i / 0xFF);
      mPaletteTable[i].green = 0xFF - ((i - 0x7f) * (i - 0x80) / 0x40);
      mPaletteTable[i].blue = ((0xFF - i) * (0xFF - i)/ 0xFF);
    }
  }
  ThermographPalette(const PNGPalette& aSource) = delete;
public:
  PNGPalette& operator= (const PNGPalette& aSource) = delete;

  static PNGPalette& getInstance() {
    static ThermographPalette instance;
    return instance;
  }
};

void createTile(uint32_t aZ, uint32_t aX, uint32_t aY,
                const HimawariStandardData& aData, DataType aType,
                uint32_t aBand);
void createTile(uint32_t aZ, uint32_t aX, uint32_t aY,
                const HimawariStandardData& aData, DataType aType,
                uint32_t aBand1, uint32_t aBand2);
void createTile(uint32_t aZ, uint32_t aX, uint32_t aY,
                const HimawariStandardData& aData, DataType aType,
                uint32_t aBand0, uint32_t aBand1, uint32_t aBand2);

void createTile(const std::string& aDir, uint32_t aZ, uint32_t aX, uint32_t aY,
                const HimawariStandardData& aData, DataType aType,
                uint32_t aBand);
void createTile(const std::string& aDir, uint32_t aZ, uint32_t aX, uint32_t aY,
                const HimawariStandardData& aData, DataType aType,
                uint32_t aBand1, uint32_t aBand2);
void createTile(const std::string& aDir, uint32_t aZ, uint32_t aX, uint32_t aY,
                const HimawariStandardData& aData, DataType aType,
                uint32_t aBand0, uint32_t aBand1, uint32_t aBand2);


void shrinkTile(uint32_t aZ, uint32_t aX, uint32_t aY,
                DataType aType,
                uint32_t aBand);
void shrinkTile(uint32_t aZ, uint32_t aX, uint32_t aY,
                DataType aType,
                uint32_t aBand0, uint32_t aBand1, uint32_t aBand2);

void createAltitudeFile(uint32_t aZ, uint32_t aX, uint32_t aY,
                        const HimawariStandardData& aData,
                        const CloudTopAltitude& aTable);

void colorchart();

} // hsd2tms
#endif
