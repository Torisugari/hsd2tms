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

#include "png.h"
#include <assert.h>
#include <algorithm>

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include "HimawariStandardData.h"
#include "TileMapService.h"
#include <sys/stat.h>

#ifdef DEBUG
// Int
#define DUMPI(_VAR_) std::cout << #_VAR_ ": " << uint32_t(_VAR_) << std::endl
// Float
#define DUMPF(_VAR_) std::cout << #_VAR_ ": " << double(_VAR_) << std::endl
// String
#define DUMPS(_VAR_) std::cout << #_VAR_ ": " << \
  std::string(_VAR_, sizeof(_VAR_)) << std::endl
#endif

namespace hsd2tms {

void writePNG(const char* aFileName, uint8_t** aLines) {
  //XXX Do error handling!
  FILE* fp = fopen(aFileName, "wb");
  png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, 0, 0, 0);
  png_infop info_ptr = png_create_info_struct(png_ptr);
  setjmp(png_jmpbuf(png_ptr));
  png_init_io(png_ptr, fp);

  info_ptr->width = 256;
  info_ptr->height = 256;
  info_ptr->bit_depth = 8;
  info_ptr->color_type = PNG_COLOR_TYPE_RGB;
  info_ptr->interlace_type = PNG_INTERLACE_NONE;
  info_ptr->compression_type = PNG_COMPRESSION_TYPE_DEFAULT;
  info_ptr->filter_type = PNG_FILTER_TYPE_DEFAULT;

  png_set_rows(png_ptr, info_ptr, aLines);
  png_write_png(png_ptr, info_ptr, PNG_TRANSFORM_IDENTITY, 0);
  png_destroy_write_struct(&png_ptr, 0);
  fclose(fp);
}

inline void composePath(std::string& aPath, 
                        uint32_t aZ, uint32_t aX, uint32_t aY, bool aMkdir) {

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

struct TileBitmap {
  uint8_t data[256][256][3];
};

void readPNG(const char* aFileName, TileBitmap& aTileBitmap) {
  FILE *fp = fopen(aFileName, "rb");
  assert(fp);

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

  assert(256 == png_get_image_width(png_ptr, info_ptr));
  assert(256 == png_get_image_height(png_ptr, info_ptr));
  assert(PNG_COLOR_TYPE_RGB == png_get_color_type(png_ptr, info_ptr));
  assert(8 == png_get_bit_depth(png_ptr, info_ptr));

  uint8_t* row = &(aTileBitmap.data[0][0][0]);
  for (int i = 0; i < 256; i++) {
    png_read_rows(png_ptr, &row, png_bytepp_NULL, 1);
    row += 256*3;
  }

  fclose(fp);
}

void shrinkTile(uint32_t aZ, uint32_t aX, uint32_t aY) {
  std::string path("./");
  composePath(path, aZ, aX, aY, true);
  std::cout << "Creating " << path << std::endl;

  uint8_t data[256][256][3];
  uint8_t* lines[256];

  // TOP-LEFT
  TileBitmap bitmap;
  std::string sourcePath("./");
  composePath(sourcePath, aZ + 1 , aX*2, aY*2, false);
  readPNG(sourcePath.c_str(), bitmap);

  for (int i = 0; i < 128; i++) {
    for (int j = 0; j < 128; j++) {
      uint32_t red = bitmap.data[i*2][j*2][0] + bitmap.data[i*2][j*2+1][0] +
                     bitmap.data[i*2+1][j*2][0] + bitmap.data[i*2+1][j*2+1][0];
      data[i][j][0] = uint8_t(red / 4);

      uint32_t green = bitmap.data[i*2][j*2][1] + bitmap.data[i*2][j*2+1][1] +
                     bitmap.data[i*2+1][j*2][1] + bitmap.data[i*2+1][j*2+1][1];
      data[i][j][1] = uint8_t(green / 4);

      uint32_t blue = bitmap.data[i*2][j*2][2] + bitmap.data[i*2][j*2+1][2] +
                     bitmap.data[i*2+1][j*2][2] + bitmap.data[i*2+1][j*2+1][2];
      data[i][j][2] = uint8_t(blue / 4);
    }
  }
  // TOP-RIGHT
  sourcePath.assign("./");
  composePath(sourcePath, aZ + 1 , aX*2 + 1, aY*2, false);
  readPNG(sourcePath.c_str(), bitmap);

  for (int i = 0; i < 128; i++) {
    lines[i] = &data[i][0][0];
    for (int j = 0; j < 128; j++) {
      uint32_t red = bitmap.data[i*2][j*2][0] + bitmap.data[i*2][j*2+1][0] +
                     bitmap.data[i*2+1][j*2][0] + bitmap.data[i*2+1][j*2+1][0];
      data[i][j+128][0] = uint8_t(red / 4);

      uint32_t green = bitmap.data[i*2][j*2][1] + bitmap.data[i*2][j*2+1][1] +
                     bitmap.data[i*2+1][j*2][1] + bitmap.data[i*2+1][j*2+1][1];
      data[i][j+128][1] = uint8_t(green / 4);

      uint32_t blue = bitmap.data[i*2][j*2][2] + bitmap.data[i*2][j*2+1][2] +
                     bitmap.data[i*2+1][j*2][2] + bitmap.data[i*2+1][j*2+1][2];
      data[i][j+128][2] = uint8_t(blue / 4);
    }
  }

  // BOTTOM-LEFT
  sourcePath.assign("./");
  composePath(sourcePath, aZ + 1 , aX*2, aY*2 + 1, false);
  readPNG(sourcePath.c_str(), bitmap);

  for (int i = 0; i < 128; i++) {
    lines[i] = &data[i][0][0];
    for (int j = 0; j < 128; j++) {
      uint32_t red = bitmap.data[i*2][j*2][0] + bitmap.data[i*2][j*2+1][0] +
                     bitmap.data[i*2+1][j*2][0] + bitmap.data[i*2+1][j*2+1][0];
      data[i+128][j][0] = uint8_t(red / 4);

      uint32_t green = bitmap.data[i*2][j*2][1] + bitmap.data[i*2][j*2+1][1] +
                     bitmap.data[i*2+1][j*2][1] + bitmap.data[i*2+1][j*2+1][1];
      data[i+128][j][1] = uint8_t(green / 4);

      uint32_t blue = bitmap.data[i*2][j*2][2] + bitmap.data[i*2][j*2+1][2] +
                     bitmap.data[i*2+1][j*2][2] + bitmap.data[i*2+1][j*2+1][2];
      data[i+128][j][2] = uint8_t(blue / 4);
    }
  }

  // BOTTOM-RIGHT
  sourcePath.assign("./");
  composePath(sourcePath, aZ + 1 , aX*2 + 1, aY*2 + 1, false);
  readPNG(sourcePath.c_str(), bitmap);

  for (int i = 0; i < 128; i++) {
    lines[i] = &data[i][0][0];
    for (int j = 0; j < 128; j++) {
      uint32_t red = bitmap.data[i*2][j*2][0] + bitmap.data[i*2][j*2+1][0] +
                     bitmap.data[i*2+1][j*2][0] + bitmap.data[i*2+1][j*2+1][0];
      data[i+128][j+128][0] = uint8_t(red / 4);

      uint32_t green = bitmap.data[i*2][j*2][1] + bitmap.data[i*2][j*2+1][1] +
                       bitmap.data[i*2+1][j*2][1] + bitmap.data[i*2+1][j*2+1][1];
      data[i+128][j+128][1] = uint8_t(green / 4);

      uint32_t blue = bitmap.data[i*2][j*2][2] + bitmap.data[i*2][j*2+1][2] +
                     bitmap.data[i*2+1][j*2][2] + bitmap.data[i*2+1][j*2+1][2];
      data[i+128][j+128][2] = uint8_t(blue / 4);
    }
  }

  for (int i = 0; i < 256; i++) {
    lines[i] = &data[i][0][0];
  }
  writePNG(path.c_str(), lines);
}

void createTile(uint32_t aZ, uint32_t aX, uint32_t aY,
                HimawariStandardData& aData) {
  TileMapService tile;
  tile.init(aZ, aX, aY);

  std::string path("./");
  composePath(path, aZ, aX, aY, true);
  std::cout << "Creating " << path << std::endl;

  static png_byte data[256][256][3] = {{{0}}};
  png_byte* lines[256];

  for (int i = 0; i < 256; i++) {
    lines[i] = &data[i][0][0];
    double latitude = tile.latitude(i);
    for (int j = 0; j < 256; j++) {
      double longitude = tile.longitude(j);
      uint32_t count = aData.mRed.dataAt(longitude, latitude);
      data[i][j][0] = (count > 1200)? 255 : int8_t(count*255/1200);

      count = aData.mGreen.dataAt(longitude, latitude);
      data[i][j][1] = (count > 1200)? 255 : int8_t(count*255/1200);

      count = aData.mBlue.dataAt(longitude, latitude);
      data[i][j][2] = (count > 1200)? 255 : int8_t(count*255/1200);
    }
  }

  writePNG(path.c_str(), lines);
}
} // hsd2tms
int main (int argc, char* argv[]) {

  hsd2tms::HimawariStandardData himawariData;
  himawariData.mSegments.reserve(30);

  for (int i = 1; i < argc; i++) {
    himawariData.append(argv[i]);
  }

  // Generate zoom-level 7 tiles.
  int32_t z = 7;
  uint32_t max = 1 << z;
  for (uint32_t x = 0; x < max; x++) {
    for (uint32_t y = 0; y < max; y++) {
      hsd2tms::createTile(z, x, y, himawariData);
    }
  }
  //XXX Release hiamawariData here so as to save our machine's memory.
  // Generate zoom-level 1 ~ 6 tiles.
  for (z--; z >= 0; z--) {
    max = 1 << z;
    for (uint32_t x = 0; x < max; x++) {
      for (uint32_t y = 0; y < max; y++) {
        hsd2tms::shrinkTile(z, x, y);
      }
    }
  }

  return 0;
}
