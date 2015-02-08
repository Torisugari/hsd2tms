#include "hsd2tms.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>

namespace hsd2tms {

struct BitmapRGB {
  uint8_t data[256][256][3];
  uint8_t* lines[256];
  BitmapRGB() {
    fillZero();
    for (int i = 0; i < 256; i++) {
      lines[i] = &data[i][0][0];
    }
  }
  void fillZero() {
    std::fill(&data[0][0][0], &data[256][0][0], 0);
  }
};

struct BitmapGrayAlpha {
  uint8_t data[256][256][2];
  uint8_t* lines[256];
  BitmapGrayAlpha() {
    fillZero();
    for (int i = 0; i < 256; i++) {
      lines[i] = &data[i][0][0];
    }
  }
  void fillZero() {
    std::fill(&data[0][0][0], &data[256][0][0], 0);
  }
};

template <typename T>
bool readPNG(const char* aFileName, T& aBitmap) {
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

  assert(256 == png_get_image_width(png_ptr, info_ptr));
  assert(256 == png_get_image_height(png_ptr, info_ptr));
#ifdef DEBUG
  uint8_t colortype = (2 == sizeof(aBitmap.data[0][0]))? 
    PNG_COLOR_TYPE_GRAY_ALPHA : PNG_COLOR_TYPE_RGB;
  assert(png_get_color_type(png_ptr, info_ptr) == colortype);
#endif
  assert(8 == png_get_bit_depth(png_ptr, info_ptr));

  uint8_t* row = &(aBitmap.data[0][0][0]);
  for (int i = 0; i < 256; i++) {
    png_read_rows(png_ptr, &row, png_bytepp_NULL, 1);
    row += sizeof(aBitmap.data[0]);
  }

  fclose(fp);
  return true;
}

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

inline void print0d(std::string& aStr, uint32_t aInt, uint32_t aOrder) {
  std::ostringstream stream;
  stream << std::setfill('0') << std::setw(aOrder) << aInt;
  aStr += stream.str();
}

inline void composeParent(std::string& aPath,
                          DataType aType,
                          uint32_t aBand) {
  aPath.assign("./");
  aPath.append(HimawariStandardData::typeTag(aType));
  print0d(aPath, aBand + 1, 2);
  aPath += "/";
}

inline void composeParent(std::string& aPath,
                          DataType aType,
                          uint32_t aBand0, uint32_t aBand1, uint32_t aBand2) {
  aPath.assign("./");
  aPath.append(HimawariStandardData::typeTag(aType));
  print0d(aPath, aBand0 + 1, 2);
  print0d(aPath, aBand1 + 1, 2);
  print0d(aPath, aBand2 + 1, 2);
  aPath += "/";
}

template <typename T>
inline void quarterCopy(T& aSource, T& aDest,
                        uint32_t aMatrixX, size_t aMatrixY) {
  const size_t colors = sizeof(aSource.data[0][0]);
  const size_t offsetX = aMatrixX * 128;
  const size_t offsetY = aMatrixY * 128;

  for (size_t i = 0; i < 128; i++) {
    for (size_t j = 0; j < 128; j++) {
      for (size_t k = 0; k < colors; k++) {
        uint32_t data = aSource.data[(i * 2)     ][(j * 2)    ][k] +
                        aSource.data[(i * 2)     ][(j * 2) + 1][k] +
                        aSource.data[(i * 2)  + 1][(j * 2)    ][k] +
                        aSource.data[(i * 2)  + 1][(j * 2) + 1][k];
        aDest.data[i + offsetY][j + offsetX][k] = uint8_t(data / 4);
      }
    }
  }
}

template <typename T>
inline bool
readAndQuaterCopy(T& aSource, T& aDest, std::string& aParent,
                  uint32_t aZ, uint32_t aX, uint32_t aY,
                  uint32_t aMatrixX, size_t aMatrixY) {
  std::string sourcePath(aParent);
  composePath(sourcePath, aZ + 1 , aX * 2 + aMatrixX, aY * 2 + aMatrixY, false);
  bool found = readPNG(sourcePath.c_str(), aSource);
  if (found) {
    quarterCopy(aSource, aDest, aMatrixX, aMatrixY);
  }
  return found;
}


void writePNG(const char* aFileName, uint8_t aColorType, uint8_t** aLines) {
  //XXX Do error handling!
  FILE* fp = fopen(aFileName, "wb");
  png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, 0, 0, 0);
  png_infop info_ptr = png_create_info_struct(png_ptr);
  setjmp(png_jmpbuf(png_ptr));
  png_init_io(png_ptr, fp);

  info_ptr->width = 256;
  info_ptr->height = 256;
  info_ptr->bit_depth = 8;
  info_ptr->color_type = aColorType;
  info_ptr->interlace_type = PNG_INTERLACE_NONE;
  info_ptr->compression_type = PNG_COMPRESSION_TYPE_DEFAULT;
  info_ptr->filter_type = PNG_FILTER_TYPE_DEFAULT;

  png_set_rows(png_ptr, info_ptr, aLines);
  png_write_png(png_ptr, info_ptr, PNG_TRANSFORM_IDENTITY, 0);
  png_destroy_write_struct(&png_ptr, 0);
  fclose(fp);
}

void createTile(uint32_t aZ, uint32_t aX, uint32_t aY,
                const HimawariStandardData& aData,
                DataType aType,
                uint32_t aBand) {
  TileMapService tile;
  tile.init(aZ, aX, aY);

  std::string path;
  composeParent(path, aType, aBand);

  composePath(path, aZ, aX, aY, true);
  std::cout << "Creating " << path << std::endl;


  if (TypeRadiation == aType) {
    uint8_t data[256][256][2] = {{{0}}};
    uint8_t* lines[256];

    for (int i = 0; i < 256; i++) {
      lines[i] = &data[i][0][0];
      double latitude = tile.latitude(i);
      for (int j = 0; j < 256; j++) {
        double longitude = tile.longitude(j);
        data[i][j][1] = aData.mBands[aBand]
                             .normalizedDataAt(longitude, latitude, aType);
      }
      writePNG(path.c_str(), PNG_COLOR_TYPE_GRAY_ALPHA, lines);
    }
  } 
  else {
    uint8_t data[256][256][3] = {{{0}}};
    uint8_t* lines[256];

    for (int i = 0; i < 256; i++) {
      lines[i] = &data[i][0][0];
      double latitude = tile.latitude(i);
      for (int j = 0; j < 256; j++) {
        double longitude = tile.longitude(j);
        uint32_t tmp = aData.mBands[aBand]
                            .normalizedDataAt(longitude, latitude, aType);

        data[i][j][0] = uint8_t(tmp);
        data[i][j][1] = uint8_t((tmp < 0x80)? (0x80 + tmp/2) : (0xFF - tmp/2));
        data[i][j][2] = uint8_t(0xFF - tmp);
      }
    }
    writePNG(path.c_str(), PNG_COLOR_TYPE_RGB, lines);
  }
}

void createTile(uint32_t aZ, uint32_t aX, uint32_t aY,
                const HimawariStandardData& aData,
                DataType aType,
                uint32_t aBand0, uint32_t aBand1, uint32_t aBand2) {
  TileMapService tile;
  tile.init(aZ, aX, aY);

  std::string path;
  composeParent(path, aType, aBand0, aBand1, aBand2);

  composePath(path, aZ, aX, aY, true);
  std::cout << "Creating " << path << std::endl;

  static png_byte data[256][256][3] = {{{0}}};
  png_byte* lines[256];

  for (int i = 0; i < 256; i++) {
    lines[i] = &data[i][0][0];
    double latitude = tile.latitude(i);
    for (int j = 0; j < 256; j++) {
      double longitude = tile.longitude(j);
      data[i][j][0] = aData.mBands[aBand2]
                           .normalizedDataAt(longitude, latitude, aType);
      data[i][j][1] = aData.mBands[aBand1]
                           .normalizedDataAt(longitude, latitude, aType);
      data[i][j][2] = aData.mBands[aBand0]
                           .normalizedDataAt(longitude, latitude, aType);
    }
  }

  writePNG(path.c_str(), PNG_COLOR_TYPE_RGB, lines);
}

void shrinkTile(uint32_t aZ, uint32_t aX, uint32_t aY,
                DataType aType,
                uint32_t aBand) {
  std::string parent;
  composeParent(parent, aType, aBand);

  std::string path(parent);
  composePath(path, aZ, aX, aY, true);
  std::cout << "Creating " << path << std::endl;

  if (TypeRadiation == aType) {
    static BitmapGrayAlpha dest;
    dest.fillZero();

    static BitmapGrayAlpha src;
    src.fillZero();

    uint8_t* (&lines)[256] = dest.lines;

    bool totalFound = readAndQuaterCopy(src, dest, parent, aZ, aX, aY, 0, 0);
    totalFound |= readAndQuaterCopy(src, dest, parent, aZ, aX, aY, 0, 1);
    totalFound |= readAndQuaterCopy(src, dest, parent, aZ, aX, aY, 1, 0);
    totalFound |= readAndQuaterCopy(src, dest, parent, aZ, aX, aY, 1, 1);

    if (totalFound) {
      writePNG(path.c_str(), PNG_COLOR_TYPE_GRAY_ALPHA, lines);
    }
  } 
  else {
    static BitmapRGB dest;
    dest.fillZero();

    static BitmapRGB src;
    src.fillZero();

    uint8_t* (&lines)[256] = dest.lines;

    bool totalFound = readAndQuaterCopy(src, dest, parent, aZ, aX, aY, 0, 0);
    totalFound |= readAndQuaterCopy(src, dest, parent, aZ, aX, aY, 0, 1);
    totalFound |= readAndQuaterCopy(src, dest, parent, aZ, aX, aY, 1, 0);
    totalFound |= readAndQuaterCopy(src, dest, parent, aZ, aX, aY, 1, 1);

    if (totalFound) {
      writePNG(path.c_str(), PNG_COLOR_TYPE_RGB, lines);
    }
  } 
}

void shrinkTile(uint32_t aZ, uint32_t aX, uint32_t aY,
                DataType aType,
                uint32_t aBand0, uint32_t aBand1, uint32_t aBand2) {
  std::string parent;
  composeParent(parent, aType, aBand0, aBand1, aBand2);

  std::string path(parent);
  composePath(path, aZ, aX, aY, true);
  std::cout << "Creating " << path << std::endl;

  static BitmapRGB dest;
  dest.fillZero();

  static BitmapRGB src;
  src.fillZero();

  uint8_t* (&lines)[256] = dest.lines;

  bool totalFound = readAndQuaterCopy(src, dest, parent, aZ, aX, aY, 0, 0);
  totalFound |= readAndQuaterCopy(src, dest, parent, aZ, aX, aY, 0, 1);
  totalFound |= readAndQuaterCopy(src, dest, parent, aZ, aX, aY, 1, 0);
  totalFound |= readAndQuaterCopy(src, dest, parent, aZ, aX, aY, 1, 1);

  if (totalFound) {
    writePNG(path.c_str(), PNG_COLOR_TYPE_RGB, lines);
  }
}

} // hsd2tms
