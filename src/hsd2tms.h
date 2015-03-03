#ifndef HSD_TO_TMS_GRAY
#define HSD_TO_TMS_GRAY
#include <sstream>
#include <iostream>
#include <iomanip>

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
} // hsd2tms
#endif
