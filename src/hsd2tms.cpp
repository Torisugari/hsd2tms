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
#include <iostream>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <fstream>
#include "hsd2tms.h"

namespace hsd2tms {

inline void composePath(std::string& aPath, 
                        uint32_t aZ, uint32_t aX, uint32_t aY, bool aMkdir) {
  if (aMkdir) {
    mkdir(aPath.c_str(), 0755);
  }

  aPath += std::to_string(aZ);
  if (aMkdir) {
    mkdir(aPath.c_str(), 0755);
  }

  aPath += "/";
  aPath += std::to_string(aX);
  if (aMkdir) {
    mkdir(aPath.c_str(), 0755);
  }

  aPath += "/";
  aPath += std::to_string(aY);
  aPath += ".png";
}


// Note: T is either |uint8_t[2]| or |uint8_t[3]|, which corresponds
//       gray+alpha bitmap or RGB bitmap respectively.
template<typename T, const uint8_t _PNG_COLOR_TYPE, int _WIDTH, int _HEIGHT>
struct TileBitmap {
  T mData[_HEIGHT][_WIDTH];
  uint8_t* mLines[_HEIGHT];
  PNGPalette mPalette;
  TileBitmap() {
    fillZero();
    for (int i = 0; i < _HEIGHT; i++) {
      mLines[i] = &mData[i][0][0];
    }
  }

  void fillZero() {
    std::fill(&mData[0][0][0], &mData[_HEIGHT][0][0], 0);
  }

  inline uint8_t colorType() const {
    return _PNG_COLOR_TYPE;
  }

  void writePNG(const char* aFileName) {
    //XXX Do error handling!
    FILE* fp = fopen(aFileName, "wb");
    png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, 0, 0, 0);
    png_infop info_ptr = png_create_info_struct(png_ptr);
    setjmp(png_jmpbuf(png_ptr));
    png_init_io(png_ptr, fp);

    info_ptr->width = _WIDTH;
    info_ptr->height = _HEIGHT;
    info_ptr->bit_depth = 8;
    info_ptr->color_type = _PNG_COLOR_TYPE;
    info_ptr->interlace_type = PNG_INTERLACE_NONE;
    info_ptr->compression_type = PNG_COMPRESSION_TYPE_DEFAULT;
    info_ptr->filter_type = PNG_FILTER_TYPE_DEFAULT;

    // Expecting static-if
    if (PNG_COLOR_MASK_PALETTE & _PNG_COLOR_TYPE) {
      png_set_tRNS(png_ptr, info_ptr, mPalette.mAlphaTable, 256, nullptr);
      png_set_PLTE(png_ptr, info_ptr, mPalette.mPaletteTable, 256);
    }

    png_set_rows(png_ptr, info_ptr, mLines);
    png_write_png(png_ptr, info_ptr, PNG_TRANSFORM_IDENTITY, 0);
    png_destroy_write_struct(&png_ptr, 0);
    fclose(fp);
  }

  bool readPNG(const char* aFileName) {
    FILE *fp = fopen(aFileName, "rb");
    if (!fp) {
      return false;
    }

    // Check PNG header.
    uint8_t header[8];
#ifdef DEBUG
    size_t read = 
#endif
    fread(header, 1, 8, fp);
    assert(read == 8);
    assert(0 == png_sig_cmp(header, 0, 8));

    png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING,
                                                 nullptr, nullptr, nullptr);
    assert(png_ptr);
    png_infop info_ptr = png_create_info_struct(png_ptr);
    assert(info_ptr);

    assert(!setjmp(png_jmpbuf(png_ptr)));
    png_init_io(png_ptr, fp);
    png_set_sig_bytes(png_ptr, 8);

    png_read_info(png_ptr, info_ptr);

    assert(_WIDTH == png_get_image_width(png_ptr, info_ptr));
    assert(_HEIGHT == png_get_image_height(png_ptr, info_ptr));
    assert(_PNG_COLOR_TYPE == png_get_color_type(png_ptr, info_ptr));
    assert(8 == png_get_bit_depth(png_ptr, info_ptr));

    // Expecting static-if
    if (PNG_COLOR_MASK_PALETTE & _PNG_COLOR_TYPE) {
      int size;
      png_color_16p value;

      uint8_t* alpha;;
      png_get_tRNS(png_ptr, info_ptr, &alpha, &size, &value);
      assert(256 == size);
      //assert(nullptr == value);
      std::memcpy(mPalette.mAlphaTable, alpha, sizeof(mPalette.mAlphaTable));

      png_color* palette;
      png_get_PLTE(png_ptr, info_ptr, &palette, &size);
      assert(256 == size);
      std::memcpy(mPalette.mPaletteTable, palette, sizeof(mPalette.mPaletteTable));
    }

    uint8_t* row = &(mData[0][0][0]);
    for (int i = 0; i < _HEIGHT; i++) {
      png_read_rows(png_ptr, &row, png_bytepp_NULL, 1);
      row += sizeof(T) * _WIDTH;
    }
    png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);
    fclose(fp);

    return true;
  }

  inline void quarterCopy(TileBitmap<T, _PNG_COLOR_TYPE, _WIDTH, _HEIGHT>& aSrc,
                          uint32_t aMatrixX, size_t aMatrixY) {
    const size_t colors = sizeof(T);
    const size_t offsetX = aMatrixX * (_WIDTH / 2);
    const size_t offsetY = aMatrixY * (_HEIGHT / 2);

    for (size_t i = 0; i < (_HEIGHT / 2); i++) {
      for (size_t j = 0; j < (_WIDTH / 2); j++) {
        for (size_t k = 0; k < colors; k++) {
          uint32_t data = aSrc.mData[(i * 2)     ][(j * 2)    ][k] +
                          aSrc.mData[(i * 2)     ][(j * 2) + 1][k] +
                          aSrc.mData[(i * 2)  + 1][(j * 2)    ][k] +
                          aSrc.mData[(i * 2)  + 1][(j * 2) + 1][k];
          mData[i + offsetY][j + offsetX][k] = uint8_t(data / 4);
        }
      }
    }
  }

  inline bool
  readAndQuaterCopy(TileBitmap<T, _PNG_COLOR_TYPE, _WIDTH, _HEIGHT>& aSrc,
                    const std::string& aParent,
                    uint32_t aZ, uint32_t aX, uint32_t aY,
                    uint32_t aMatrixX, size_t aMatrixY) {
    std::string sourcePath(aParent);
    composePath(sourcePath, aZ + 1 , aX * 2 + aMatrixX, aY * 2 + aMatrixY, false);
    bool found = aSrc.readPNG(sourcePath.c_str());
    if (found) {
      quarterCopy(aSrc, aMatrixX, aMatrixY);
    }
    return found;
  }

  void shrinkTile(TileBitmap<T, _PNG_COLOR_TYPE, _WIDTH, _HEIGHT>& aSrc,
                  const std::string& aParent,
                  uint32_t aZ, uint32_t aX, uint32_t aY) {

    std::string path(aParent);
    composePath(path, aZ, aX, aY, true);
    std::cout << "Creating " << path << std::endl;

    bool
    totalFound  = readAndQuaterCopy(aSrc, aParent, aZ, aX, aY, 0, 0);
    totalFound |= readAndQuaterCopy(aSrc, aParent, aZ, aX, aY, 0, 1);
    totalFound |= readAndQuaterCopy(aSrc, aParent, aZ, aX, aY, 1, 0);
    totalFound |= readAndQuaterCopy(aSrc, aParent, aZ, aX, aY, 1, 1);

    if (totalFound) {
      // Expecting static-if
      if (PNG_COLOR_MASK_PALETTE & _PNG_COLOR_TYPE) {
        mPalette = aSrc.mPalette;
      }
      writePNG(path.c_str());
    }
  }
};

typedef TileBitmap<uint8_t[3], PNG_COLOR_TYPE_RGB, 256, 256> RGBTile;
typedef TileBitmap<uint8_t[2], PNG_COLOR_TYPE_GA, 256, 256> GATile;
typedef TileBitmap<uint8_t[1], PNG_COLOR_TYPE_PALETTE, 256, 256> PaletteTile;

void colorchart() {
  TileBitmap<uint8_t[1], PNG_COLOR_TYPE_PALETTE, 256, 10> bitmap;
  bitmap.mPalette = ThermographPalette::getInstance();
  for (auto& row: bitmap.mData) {
    for (int i = 0; i < 256; i++) {
      row[i][0] = i;
    }
  }
  bitmap.writePNG("./thermo.png");
}
static const double kMaxRadiation[16] = 
  {300., 280., 260., 300., 40., 10., 1., 1.5,
   3., 3., 3., 3., 3., 3., 3., 3.};
#if 0
band: 0
championR: 596.924
band: 1
championR: 570.674
band: 2
championR: 526.357
band: 3
championR: 313.535
band: 4
championR: 42.1191
band: 5
championR: 10.1764
band: 6
championR: 0.981368
band: 7
championR: 1.44067
band: 8
championR: 2.85773
#endif

inline uint8_t normalizeRadiation(double aRadiation, uint32_t aBand) {
  if (aRadiation <= 0) {
    return 0;
  }
  const double& max = kMaxRadiation[(aBand & 0xF)];
  return (max < aRadiation)? 0xFF : uint8_t((aRadiation * 255.) / max);
}

inline uint8_t normalizeTemperature(double aTemperature) {
  static const double kMin = 273.15 - 40.;
  static const double kMax = 273.15 + 40.;
  static const double kRange = kMax - kMin;

  // clamp
  if (aTemperature < kMin) {
    return 0x00;
  }
  else if (kMax < aTemperature) {
    return 0xFF;
  }
  return uint8_t((aTemperature - kMin) * 255. / kRange);
}

uint8_t normalizedData(const HimawariStandardData& aData,
                       double aLongitude, double aLatitude,
                       DataType aType, uint32_t aBand) {
  double rawdata;
  switch(aType) {
  case TypeRadiation:
    rawdata = aData.mBands[aBand].radiationAt(aLongitude, aLatitude);
    return normalizeRadiation(rawdata, aBand);
    break;
  case TypeTemperature:
    rawdata = aData.mBands[aBand].temperatureAt(aLongitude, aLatitude);
    return normalizeTemperature(rawdata);
    break;
  default:
    break;
  }
  return 0;
}

uint8_t normalizedData(const HimawariStandardData& aData,
                       double aLongitude, double aLatitude,
                       DataType aType, uint32_t aBand0, uint32_t aBand1) {
  double rawdata0, rawdata1;
  switch(aType) {
  case TypeDust:
    rawdata0 = aData.mBands[aBand0].radiationAt(aLongitude, aLatitude);
    rawdata1 = aData.mBands[aBand1].radiationAt(aLongitude, aLatitude);
    return normalizeRadiation(rawdata0 - rawdata1, aBand0);
    break;
  default:
    break;
  }
  return 0;
}

void createTile(const std::string& aDir, uint32_t aZ, uint32_t aX, uint32_t aY,
                const HimawariStandardData& aData,
                DataType aType,
                uint32_t aBand) {
  TileMapService tile;
  tile.init(aZ, aX, aY);

  std::string path(aDir);
  composePath(path, aZ, aX, aY, true);
  std::cout << "Creating " << path << std::endl;

  PaletteTile bitmap;
  static const PNGPalette pink = SingleColorPalette(0xFF, 0xCC, 0xCC);
  PNGPalette& thermo = ThermographPalette::getInstance();

  switch (aType) {
  case TypeTemperature:
    bitmap.mPalette = thermo;
    break;
  default:
    bitmap.mPalette = pink;
    break;
  }

  for (int i = 0; i < 256; i++) {
    double latitude = tile.latitude(i);
    for (int j = 0; j < 256; j++) {
      double longitude = tile.longitude(j);
      bitmap.mData[i][j][0] = normalizedData(aData, longitude, latitude, aType, aBand);
    }
  }
  bitmap.writePNG(path.c_str());
}

void createTile(const std::string& aDir, uint32_t aZ, uint32_t aX, uint32_t aY,
                const HimawariStandardData& aData,
                DataType aType,
                uint32_t aBand0, uint32_t aBand1) {
  TileMapService tile;
  tile.init(aZ, aX, aY);

  std::string path(aDir);
  composePath(path, aZ, aX, aY, true);
  std::cout << "Creating " << path << std::endl;

  PaletteTile bitmap;
  static const PNGPalette yellow = SingleColorPalette(0xFF, 0xFF, 0x00);
  static const PNGPalette pink = SingleColorPalette(0xFF, 0xCC, 0xCC);

  switch (aType) {
  case TypeDust:
    bitmap.mPalette = yellow;
    break;
  default:
    bitmap.mPalette = pink;
    break;
  }

  for (int i = 0; i < 256; i++) {
    double latitude = tile.latitude(i);
    for (int j = 0; j < 256; j++) {
      double longitude = tile.longitude(j);
      bitmap.mData[i][j][0] = normalizedData(aData, longitude, latitude, aType, aBand0, aBand1);
    }
  }
  bitmap.writePNG(path.c_str());
}

void createTile(const std::string& aDir, uint32_t aZ, uint32_t aX, uint32_t aY,
                const HimawariStandardData& aData,
                DataType aType,
                uint32_t aBand0, uint32_t aBand1, uint32_t aBand2) {
  TileMapService tile;
  tile.init(aZ, aX, aY);

  std::string path(aDir);
  composePath(path, aZ, aX, aY, true);
  std::cout << "Creating " << path << std::endl;

  RGBTile bitmap;

  for (int i = 0; i < 256; i++) {
    double latitude = tile.latitude(i);
    for (int j = 0; j < 256; j++) {
      double longitude = tile.longitude(j);
      bitmap.mData[i][j][0] = normalizedData(aData, longitude, latitude, aType, aBand2);
      bitmap.mData[i][j][1] = normalizedData(aData, longitude, latitude, aType, aBand1);
      bitmap.mData[i][j][2] = normalizedData(aData, longitude, latitude, aType, aBand0);
    }
  }
  bitmap.writePNG(path.c_str());
}

void createTile(uint32_t aZ, uint32_t aX, uint32_t aY,
                const HimawariStandardData& aData,
                DataType aType,
                uint32_t aBand) {
  std::string path;
  DirNameProvider::compose(path, aType, aBand);
  createTile(path, aZ, aX, aY, aData, aType, aBand);
}

void createTile(uint32_t aZ, uint32_t aX, uint32_t aY,
                const HimawariStandardData& aData,
                DataType aType,
                uint32_t aBand0, uint32_t aBand1) {
  std::string path;
  DirNameProvider::compose(path, aType, aBand0);
  createTile(path, aZ, aX, aY, aData, aType, aBand0, aBand1);
}

void createTile(uint32_t aZ, uint32_t aX, uint32_t aY,
                const HimawariStandardData& aData,
                DataType aType,
                uint32_t aBand0, uint32_t aBand1, uint32_t aBand2) {
  std::string path;
  DirNameProvider::compose(path, aType, aBand0, aBand1, aBand2);
  createTile(path, aZ, aX, aY, aData, aType, aBand0, aBand1, aBand2);
}
             
void shrinkTile(uint32_t aZ, uint32_t aX, uint32_t aY,
                DataType aType,
                uint32_t aBand) {
  std::string parent;
  DirNameProvider::compose(parent, aType, aBand);

  static PaletteTile src;
  static PaletteTile dst;
  dst.fillZero();
  dst.shrinkTile(src, parent, aZ, aX, aY);
}

void shrinkTile(uint32_t aZ, uint32_t aX, uint32_t aY,
                DataType aType,
                uint32_t aBand0, uint32_t aBand1, uint32_t aBand2) {
  std::string parent;
  DirNameProvider::compose(parent, aType, aBand0, aBand1, aBand2);

  static RGBTile src;
  static RGBTile dst;
  dst.fillZero();
  dst.shrinkTile(src, parent, aZ, aX, aY);
}

inline void composeJSPath(std::string& aPath, 
                          uint32_t aZ, uint32_t aX, uint32_t aY, bool aMkdir) {

  if (aMkdir) {
    mkdir(aPath.c_str(), 0755);
  }

  aPath += std::to_string(aZ);
  if (aMkdir) {
    mkdir(aPath.c_str(), 0755);
  }

  aPath += "/";
  aPath += std::to_string(aX);
  if (aMkdir) {
    mkdir(aPath.c_str(), 0755);
  }

  aPath += "/";
  aPath += std::to_string(aY);
  aPath += ".js";
}

void createAltitudeFile(uint32_t aZ, uint32_t aX, uint32_t aY,
                        const HimawariStandardData& aData,
                        const CloudTopAltitude& aTable) {
  TileMapService tile;
  tile.init(aZ, aX, aY);

  std::string path("./rad010203/");
  composeJSPath(path, aZ, aX, aY, true);
  std::ofstream output(path);

  output << "var altitude = new Float64Array([\n";
  for (int i = 0; i < 256; i++) {
    double latitude = tile.latitude(i);
    for (int j = 0; j < 256; j++) {
      double longitude = tile.longitude(j);
      double tb11, tb12;
      aData.brightnessTemperaturesAt(longitude, latitude, tb11, tb12);
      double zenith = aData.mSegments[0].zenith(longitude, latitude) * 180. /M_PI;
      double altitude = aTable.linear(tb11, tb12, zenith);
      if (altitude < 0.) {
        altitude = 0.;
      }
      output << altitude << "," << std::endl;
    }
  }
  output << "]);\n";
}
} // hsd2tms
