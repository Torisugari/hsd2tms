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
#ifndef TILE_MAP_SERVICE
#define TILE_MAP_SERVICE
#include <math.h>
#include <vector>
namespace hsd2tms {
class TileMapService {
private:
  double mDotWidth;
  double mDotHeight;
  double mLeft;
  double mTop;

  static inline double gudermannian(double aY) {
    return atan(sinh(aY));
  }

  static inline double lambertian(double aLatitude) {
    return asinh(tan(aLatitude));
  }
public:
  void init(uint32_t aZ, uint32_t aX, uint32_t aY) {
    uint32_t scale = 1 << aZ;
    assert(aX < scale && aY < scale);

    static const double globalLeft   = M_PI * -1.;
    static const double globalTop    = lambertian(M_PI * 85.05 / 180.);

    static const double globalWidth  = M_PI * 2.;
    static const double globalHeight = lambertian(M_PI * 85.05 / 180.) * 2.;

    double imageWidth = globalWidth / double(scale);
    double imageHeight = globalHeight / double(scale);

    mDotWidth = imageWidth / 256.;
    mDotHeight = imageHeight / 256.;

    mLeft = globalLeft + (imageWidth * aX) + (mDotWidth * 0.5);
    mTop = globalTop - (imageHeight * aY) - (mDotHeight * 0.5);
  }

  double latitude (uint32_t aRow) const {
    double y = mTop - (mDotHeight * double(aRow));
    return gudermannian(y);
  }

  double longitude (uint32_t aColmun) const {
    return mLeft + (mDotWidth * double(aColmun));
  }
};


} //hsd2tms
#endif
