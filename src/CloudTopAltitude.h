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
#ifndef CLOUD_TOP_ALTITUDE
#define CLOUD_TOP_ALTITUDE
#include <stdio.h>
#include <assert.h>
namespace hsd2tms {
struct CloudTopAltitude {
  /*
    According to what I look into the bin file briefly with a hex editor,
    the data seems |float[7][131][131][6]| rather than |float[131][131][6][7]|.
   */

  float mTable[7][131][131][6];

  CloudTopAltitude() {
    assert(4 == sizeof(float));

    FILE *fp = fopen(CTOP_LUT_MT2_V20_PATH, "rb");
    assert(fp);
#ifndef NDEBUG
    size_t read = 
#endif
    fread(mTable, 1, sizeof(mTable), fp);
    assert(sizeof(mTable) == read);
    fclose(fp);
  }

  // "Nearest" approach
  double nearest(double aTb11, double aTb12, double aAngle) const {
    int32_t x = int32_t(aTb11 - 180. + 0.5);
    if (x < 0) {
      x = 0;
    }
    else if (130 < x) {
      x = 130;
    }

    int32_t y = int32_t(((aTb11 - aTb12) * 10.) + 40. + 0.5);
    if (y < 0) {
      y = 0;
    }
    else if (130 < y) {
      y = 130;
    }

    int32_t z = int32_t(((aAngle - 7.5) / 15.) + 0.5);
    if (z < 0) {
      z = 0;
    }
    else if (5 < z) {
      z = 5;
    }

    return double(mTable[1][x][y][z]);
  }

  static inline void cutoff(const double aValue, const uint32_t aSize,
                            uint32_t& aIndex, double& aDelta){
    if (aValue <= 0.) {
      aIndex = 0;
      aDelta = 0.;
    }
    else if (double(aSize - 1) <= aValue) {
      aIndex = aSize - 2;
      aDelta = 1.;
    }
    else {
      // 0 < aValue < (aSize - 1)
      aIndex = uint32_t(aValue);
      aDelta = aValue - double(aIndex);
    }

    assert(aIndex <= (aSize - 2));
    return;
  }

  // Tri-Linear interpolation
  double linear(double aTb11, double aTb12, double aAngle) const {
    double x = aTb11 - 180.;
    double y = ((aTb11 - aTb12) * 10.) + 40.;
    double z = (aAngle - 7.5) / 15.;

    uint32_t iX, iY, iZ;
    double dX, dY, dZ;
    cutoff(x, 131, iX, dX);
    cutoff(y, 131, iY, dY);
    cutoff(z, 6, iZ, dZ);

    double linearXY[2][2];
    double linearX[2];
    double value0, value1;

    for (uint32_t i = 0; i < 2; i++) {
      for (uint32_t j = 0; j < 2; j++) {
        value0 = double(mTable[1][iX + i][iY + j][iZ + 0]);
        value1 = double(mTable[1][iX + i][iY + j][iZ + 1]);
        linearXY[i][j] = value0 + ((value1 - value0) * dZ);
      }
      value0 = linearXY[i][0];
      value1 = linearXY[i][1];
      linearX[i] = value0 + ((value1 - value0) * dY);
    }

    return linearX[0] + ((linearX[1] - linearX[0]) * dX);
  }
};

} // hsd2tms
#endif
