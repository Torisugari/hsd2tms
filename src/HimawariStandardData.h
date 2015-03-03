#ifndef HIMAWARI_STANDARD_DATA
#define HIMAWARI_STANDARD_DATA
#include <math.h>
#include <vector>
#include <algorithm>
#include <assert.h>
namespace hsd2tms {
#pragma pack(1)

struct CommonBlockHead {
  uint8_t  mBlockID;           
  uint16_t mBlockSize;         
};

struct CommonBlockFoot {
  char     reserved[40];
};

// ID = 1
struct BasicInfoBlockBody {
  uint16_t mTotalBlockNum;
  uint8_t  mIsBigEndian;
  char     mSatelliteName[16];
  char     mCenterName[16];
  char     mObsType1[4];
  char     mObsType2[2];
  uint16_t mTimeLine;
  double_t mObsStartTime;
  double_t mObsEndTime;
  double_t mFileCreationMJD;
  uint32_t mHeaderSize;
  uint32_t mDataSize;
  uint8_t  mFlag1;
  uint8_t  mFlag2;
  uint8_t  mFlag3;
  uint8_t  mFlag4;
  char     mVersionName[32];
  char     mFileName[128];
};

// ID = 2
struct DataInfoBlockBody{
  uint16_t mBitsPerSample;
  uint16_t mWidth;           // Columns
  uint16_t mHeight;          // Rows
  uint8_t  mCompressionFlag; // Compression = 0: none, 1:gzip 2:b2z
};

// ID = 3
struct ProjectionInfoBlockBody{
  double_t mSubLon;   // subsatellite longitude in degree.
  uint32_t mCFAC;     // Column factor
  uint32_t mLFAC;     // Line factor
  float_t  mCOFF;     // Column offset
  float_t  mLOFF;     // Line offset
  double_t mRs;       // Distance
  double_t mReq;      // Equatorial radius
  double_t mRpol;     // Polar radius
  double_t mConst1;   // (mReq^2 - mRpol^2) / mReq^2
  double_t mConst2;   // mRpol^2 / mReq^2 | i.e. mConst1 + mConst2 = 1.0
  double_t mConst3;   // mReq^2 / mRpol^2 | i.e. mConst2 * mConst3 = 1.0
  double_t mConst4;
  uint16_t mResampling;
  uint16_t mResamplingSize;
  void lonlatToXY(double aLongitude, double aLatitude,
                  double& aX, double& aY) const {
    // See <http://2014.cgms-info.org/documents/cgms-lrit-hrit-global-specification-%28v2-8-of-30-oct-2013%29.pdf>
    // "Coordination Group for Meteorological Satellites 
    //    LRIT/HRIT Global Specification" Written by EUMETSAT on behalf of CGMS
    //
    // '4.4.3 Projection Functions'
    //
    // and comments in the sample code by MSC. Since I have no idea how to
    // improove the sample code, I'd like to paste the copyright notice as well.
#if 0
<http://www.data.jma.go.jp/mscweb/ja/himawari89/space_segment/spsg_sample.html>
<http://www.data.jma.go.jp/mscweb/en/himawari89/space_segment/hsd_sample/sample_codes_11.zip>
histd_pixlin2lonlat.c:
/* ----------------------------------------------------------------------------
	Sample source code for reading Himawari Standard Data

	Copyright (C) 2014 MSC (Meteorological Satellite Center) of JMA

	Disclaimer	:
		MSC does not guarantee regarding the correctness, accuracy, reliability,
		or any other aspect regarding use of these sample codes.

	Detail of Himawari Standard Format : 
		For data structure of Himawari Standard Format, prelese refer to MSC
		Website and Himawari Standard Data User's Guid.

		MSC Website
		http://mscweb.kishou.go.jp/index.htm

		Himawari Standard Data User's Guid
		http://mscweb.kishou.go.jp/himawari89/space_segment/hsd_sample/HS_D_users_guide_en.pdf

	History
		April, 2014  First release

---------------------------------------------------------------------------- */
# include <stdio.h>
# include <math.h>
# include "hisd.h"

#define DEGTORAD (M_PI/180.0)
#define RADTODEG (180.0/M_PI)
#define SCLUNIT  1.525878906250000e-05 	// (= 2^-16)  scaling function 

#define  NORMAL_END  0
#define  ERROR_END 100

/* ----------------------------------------------------------------------------
	Normalized Geostationary Projection is adopted as defined in LRIT/HRIT
	Global Specification Section 4.4. 
	The projection describes the view from the satellite to an idealized earth.

	LRIT/HRIT Global Specification
		http://www.cgms-info.org/publications/technical-publications
 ----------------------------------------------------------------------------*/

/* ----------------------------------------------------------------------------
	lonlat_to_pixlin()
 ----------------------------------------------------------------------------*/
int lonlat_to_pixlin(HisdHeader *head,double lon,double lat,
	float *pix,float *lin){

	// (1) init
	*pix = -9999.0;	// invalid value
	*lin = -9999.0;
	// (2) check latitude
	if(lat < -90.0 || 90.0 < lat ){
		return(ERROR_END);
	}
	// (3) check longitude
	while(lon > 180.0){ lon-=360.0; } // [deg]
	while(lon <-180.0){ lon+=360.0; } // [deg]
	// (4) degree to radian
	lon = lon * DEGTORAD; // [rad]
	lat = lat * DEGTORAD; // [rad]
	// (5) geocentric latitude
	// Global Specification 4.4.3.2
	// phi = arctan( (Rpol^2)/(Req^2) * tan(lat) )
	// 
	// (Rpol^2)/(Req^2) = head->proj->projParam2
	double phi = atan( head->proj->projParam2 * tan(lat) );
	// (6) The length of Re
	// Re = (Rpol) / sqrt( 1 - (Req^2 - Rpol^2) / Req^2 * cos^2(phi) )
	//
	// Rpol = head->proj->polrRadius
	// (Req^2 - Rpol^2) / Req^2 = head->proj->projParam1
	double Re = (head->proj->polrRadius) /
					sqrt(1 - head->proj->projParam1 * cos(phi) * cos(phi)) ;
	// (7) The cartesian components of the vector rs result as follows:
	// r1 = h - Re * cos(phi) * cos(Le-Ld)
	// r2 =    -Re * cos(phi) * sin(Le-Ld)
	// r3 =     Re * sin(phi)
	//
	// Le : longitude
	// Ld : sub_lon = head->proj->subLon
	// h  : distance from Earth's center to satellite (=head->proj->satDis)
	double r1 = head->proj->satDis - Re * cos(phi)
		 		* cos( lon - head->proj->subLon * DEGTORAD );
	double r2 = - Re * cos(phi)
				* sin( lon - head->proj->subLon * DEGTORAD );
	double r3 = Re * sin(phi);
	// (8) check seeablibity
	double vx = Re * cos(phi) * cos( lon - head->proj->subLon * DEGTORAD );
	if(0 < -r1 * vx - r2 * r2 + r3 * r3){
		return(ERROR_END);
	}
	// (9) The projection function is as follows:
	// x  = arctan(-r2/r1)
	// y  = arcsin(r3/rn)
	// rn = sqrt(r1^2 + r2^2 + r3^2)
	double rn = sqrt(r1*r1 + r2*r2 + r3*r3);
	double x = atan2(-r2,r1) * RADTODEG;
	double y = asin(-r3/rn) * RADTODEG;
	// (10)
	// Global Specification 4.4.4
	// c  = COFF + nint(x * 2^-16 * CFAC)
	// l  = LOFF + nint(y * 2^-16 * LFAC)
	float c = head->proj->coff + x * SCLUNIT * head->proj->cfac;
	float l = head->proj->loff + y * SCLUNIT * head->proj->lfac;

	*pix = c;
	*lin = l;

	return(NORMAL_END);
}
#endif
    aLongitude -= mSubLon * M_PI / 180.0;
    double phi = atan(mConst2 * tan(aLatitude));
    double Re = (mRpol) /sqrt(1. - mConst1 * cos(phi) * cos(phi));
    double r1 = mRs - Re * cos(phi) * cos(aLongitude);
    double r2 = -1. * Re * cos(phi) * sin(aLongitude);
    double r3 = Re * sin(phi);
    double vx = Re * cos(phi) * cos(aLongitude);
    if(0 < (-1. * r1 * vx) - (r2 * r2) + (r3 * r3)) {
      return;
    }
    double rn = sqrt(r1*r1 + r2*r2 + r3*r3);
    double x = atan2(-r2, r1);
    double y = asin(-r3 / rn);
    aX = mCOFF + (x * mCFAC * 1.525878906250000e-05 * 180.0 / M_PI);
    aY = mLOFF + (y * mLFAC * 1.525878906250000e-05 * 180.0 / M_PI);
  }
};

// ID = 4
struct NavigationInfoBlockBody{
  double_t mMJD;          //Modified Julius Day
  double_t mSSPLongitude; //Sub-Satellite Soint
  double_t mSSPLatitude;  //
  double_t mDistance;
  double_t mNadirLongitude;
  double_t mNadirLatitude;
  double_t mSunX;
  double_t mSunY;
  double_t mSunZ;
  double_t mMoonX;
  double_t mMoonY;
  double_t mMoonZ;
};

//ID = 5
struct CalibrationInfoBlockBodyInfrared {
  double_t mC0;
  double_t mC1;
  double_t mC2;
  double_t mC02;
  double_t mC12;
  double_t mC22;
  double_t mC;
  double_t mH;
  double_t mK;
};

struct CalibrationInfoBlockBodyVisible {
  double_t rad2albedo;
  char     padding[64];
};

struct CalibrationInfoBlockBody {
  uint16_t mBand;
  double_t mWaveLength;
  uint16_t mValidBits;
  uint16_t mErrorCount;
  uint16_t mOutCount;
  double_t mCoefficient;
  double_t mConstant;
  union u {
    CalibrationInfoBlockBodyInfrared infrared;
    CalibrationInfoBlockBodyVisible visible;
  } u;
};

// ID = 6
struct InterCalibrationInfoBlockBody {
  double_t d1;
  double_t d2;
  double_t d3;
  double_t d4;
  double_t d5;
  double_t d6;
  double_t d7;
  double_t d8;
  char mInfo[64];
  char reserved[88];
};

// ID = 7
struct SegmentInfoBlockBody {
  int8_t   mTotal;
  int8_t   mIndex;
  uint16_t mSegmentOffsetY;
};

// ID = 8
struct PositionInfoBlockBody {
  float_t mCenterX;
  float_t mCenterY;
  double_t mDeltaAngle; //micro radian
  uint16_t mCount;
};

// ID = 9
struct TimeInfoBlockBody {
  uint16_t mCount;
};

// ID = 10
struct ErrorInfoBlockBody {
  uint16_t mCount;
};

// ID = 11
struct ReservedBlockBody{
  char reserved[216];
};

struct BasicInfoBlock {
  CommonBlockHead head;
  BasicInfoBlockBody body;
  CommonBlockFoot foot;
};

struct DataInfoBlock {
  CommonBlockHead head;
  DataInfoBlockBody body;
  CommonBlockFoot foot;
};

struct ProjectionInfoBlock{
  CommonBlockHead head;
  ProjectionInfoBlockBody body;
  CommonBlockFoot foot;
};

struct NavigationInfoBlock{
  CommonBlockHead head;
  NavigationInfoBlockBody body;
  CommonBlockFoot foot;
};

struct CalibrationInfoBlock {
  CommonBlockHead head;
  CalibrationInfoBlockBody body;
  CommonBlockFoot foot;
};

struct InterCalibrationInfoBlock {
  CommonBlockHead head;
  InterCalibrationInfoBlockBody body;
  CommonBlockFoot foot;
};
struct SegmentInfoBlock {
  CommonBlockHead head;
  SegmentInfoBlockBody body;
  CommonBlockFoot foot;
};

struct PositionInfoBlock {
  CommonBlockHead head;
  PositionInfoBlockBody body;
  CommonBlockFoot foot;
  struct Item {
    uint16_t mIntY;
    float_t  mDeltaX;
    float_t  mDeltaY;
  };
  std::vector<Item> array;

  void rotate(double& aX, double& aY, bool aIsBackward) const {
    double centerX = body.mCenterX;
    double centerY = body.mCenterY;
    double theta = body.mDeltaAngle / 1000000.;

    if (aIsBackward) {
      theta *= -1.;
    }

    // [Absolute postion] -> [Relative position from the center];
    double x = aX - centerX;
    double y = aY - centerY;

    // Rotate
    double sinTheta = sin(theta);
    double cosTheta = cos(theta);
    /*
      | cos(theta) -sin(theta) |   | x |     | (xcos(theta) + -ysin(theta))|
      |                        | * |   |  =  |                             |
      | sin(theta)  cos(theta) |   | y |     | (xsin(theta) +  ycos(theta))|
     */

    double x2 = (x * cosTheta) - (y * sinTheta);
    double y2 = (x * sinTheta) + (y * cosTheta);

    // [Relative position from the center] -> [Absolute position].
    aX = x2 + centerX;
    aY = y2 + centerY;
  }

  struct ForwardCompare {
    ForwardCompare(){}
    bool operator() (const Item& aItem, const double& aDouble) {
      return double(aItem.mIntY) < aDouble;
    }

    bool operator() (const double& aDouble, const Item& aItem) {
      return  aDouble < double(aItem.mIntY);
    }
  };

  void forwardShift(double& aX, double& aY) const {
    if (array.size() < 1 || aY < double(array[0].mIntY)) {
      // Noting to modify;
      return;
    }
    static const ForwardCompare comp;
    auto itr = lower_bound(array.begin(), array.end(), aY, comp);
    aX += itr->mDeltaX;
    aY += itr->mDeltaY;
  }

  struct BackwardCompare {
    BackwardCompare(){}
    bool operator() (const Item& aItem, const double& aDouble) {
      return (double(aItem.mIntY) + aItem.mDeltaY) < aDouble;
    }

    bool operator() (const double& aDouble, const Item& aItem) {
      return  aDouble < (double(aItem.mIntY) + aItem.mDeltaY);
    }
  };

  void backwardShift(double& aX, double& aY) const {
    if (array.size() < 1 || aY < (double(array[0].mIntY) + array[0].mDeltaY)) {
      // Noting to modify;
      return;
    }
    static const BackwardCompare bwdcomp;
    //XXX I'm not too sure [aItem.mIntY + aItem.mDeltaY] is sorted...
    auto itr = lower_bound(array.begin(), array.end(), aY, bwdcomp);
    aX -= itr->mDeltaX;
    aY -= itr->mDeltaY;
  }
};

struct TimeInfoBlock {
  CommonBlockHead head;
  TimeInfoBlockBody body;
  CommonBlockFoot foot;
  struct Item {
    uint16_t  mIntY;
    double_t  obsMJD;
  };
  std::vector<Item> array;
};

struct ErrorInfoBlock {
  struct _ErrorInfoBlockHead {
    uint8_t  mBlockID;
    uint32_t mBlockSize;         //Why 4bytes while the others are 2bytes?
  } head;

  ErrorInfoBlockBody body;
  CommonBlockFoot foot;
  struct Item {
    uint16_t  mIntY;
    uint16_t  errPixNum;
  };
  std::vector<Item> array;
};

struct ReservedBlock {
  CommonBlockHead head;
  ReservedBlockBody body;
  CommonBlockFoot foot;
};

#pragma pack()

struct HimawariStandardDataSegment {
  BasicInfoBlock mBasicInfoBlock;
  DataInfoBlock mDataInfoBlock;
  ProjectionInfoBlock mProjectionInfoBlock;
  NavigationInfoBlock mNavigationInfoBlock;
  CalibrationInfoBlock mCalibrationInfoBlock;
  InterCalibrationInfoBlock mInterCalibrationInfoBlock;
  SegmentInfoBlock mSegmentInfoBlock;
  PositionInfoBlock mPositionInfoBlock;
  TimeInfoBlock mTimeInfoBlock;
  ErrorInfoBlock mErrorInfoBlock;
  ReservedBlock mReservedBlock;
  std::vector<uint16_t> mData;

  bool isInfrared() const {
    bool isHimawari7 = mBasicInfoBlock.body.mFlag1 & 0x01;
    return isHimawari7? (band() >= 2) : (band() >= 7);
  }

  template <typename T>
  static size_t readBlock(FILE* aFp, T& aBlock, uint32_t aBlockID) {
    size_t totalRead = 0;
    size_t read = fread(&aBlock.head, 1, sizeof(aBlock.head), aFp);
    totalRead += read;
    assert(sizeof(aBlock.head) == read);
    assert(aBlockID == aBlock.head.mBlockID);
    assert(sizeof(aBlock) == aBlock.head.mBlockSize);

    read = fread(&aBlock.body, 1, sizeof(aBlock.body), aFp);
    totalRead += read;
    assert(sizeof(aBlock.body) == read);

    read = fread(&aBlock.foot, 1, sizeof(aBlock.foot), aFp);
    totalRead += read;
    assert(sizeof(aBlock.foot) == read);

    return totalRead;
  }

  template <typename T>
  static size_t readBlockAndArray(FILE* aFp, T& aBlock, uint32_t aBlockID) {
    size_t totalRead = 0;
    size_t read = fread(&aBlock.head, 1, sizeof(aBlock.head), aFp);
    totalRead += read;
    assert(sizeof(aBlock.head) == read);
    assert(aBlockID == aBlock.head.mBlockID);

    read = fread(&aBlock.body, 1, sizeof(aBlock.body), aFp);
    totalRead += read;
    assert(sizeof(aBlock.body) == read);

    uint32_t size = uint32_t(aBlock.body.mCount);

    aBlock.array.resize(size);

    uint32_t i;
    for (i = 0; i < size; i++) {
      read = fread(&aBlock.array[i], 1, sizeof(aBlock.array[i]), aFp);
      totalRead += read;
      assert(sizeof(aBlock.array[i]) == read);
    }
    read = fread(&aBlock.foot, 1, sizeof(aBlock.foot), aFp);
    totalRead += read;
    assert(sizeof(aBlock.foot) == read);

    assert(totalRead == aBlock.head.mBlockSize);

    return totalRead;
  }

  void readFile(const char* aFileName) {
    FILE *fp = fopen(aFileName, "rb");
    assert(fp);

    size_t read = readBlock(fp, mBasicInfoBlock, 1);
    read += readBlock(fp, mDataInfoBlock, 2);
    read += readBlock(fp, mProjectionInfoBlock, 3);
    read += readBlock(fp, mNavigationInfoBlock, 4);
    read += readBlock(fp, mCalibrationInfoBlock, 5);
    read += readBlock(fp, mInterCalibrationInfoBlock, 6);
    read += readBlock(fp, mSegmentInfoBlock, 7);
    read += readBlockAndArray(fp, mPositionInfoBlock, 8);
    read += readBlockAndArray(fp, mTimeInfoBlock, 9);
    read += readBlockAndArray(fp, mErrorInfoBlock, 10);
    read += readBlock(fp, mReservedBlock, 11);

    assert(mBasicInfoBlock.body.mHeaderSize == read);

    mData.resize(mBasicInfoBlock.body.mDataSize / 2);
    read = fread(&(mData[0]), 1, mBasicInfoBlock.body.mDataSize, fp);
    assert(mBasicInfoBlock.body.mDataSize == read);

    fclose(fp);
  }

  uint16_t countAt(int32_t aX, int32_t aY) {
    aY -= mSegmentInfoBlock.body.mSegmentOffsetY;
    if ((aY < 0 || mDataInfoBlock.body.mHeight <= aY) ||
        (aX < 0 || mDataInfoBlock.body.mWidth  <= aX)) {
      return 0;
    }
    int32_t index = (aY * mDataInfoBlock.body.mWidth) + aX;
    if ( index < 0 || int32_t(mData.size()) <= index) {
      return 0;
    }
    return mData[index];
  }
  uint8_t band() const {
    return mCalibrationInfoBlock.body.mBand;
  }

  inline static double radius(double aLatitude) {
    const double req = 6378.1370;
    const double rpl = 6356.7523;
    double rsq = pow(req * cos(aLatitude), 2) + pow(rpl * sin(aLatitude), 2);
    return sqrt(rsq);
  }

  double zenith(double aLongitude, double aLatitude) const {
    const double& sublon = mNavigationInfoBlock.body.mSSPLongitude;
    const double& sublat = mNavigationInfoBlock.body.mSSPLatitude;
    const double& d =  mNavigationInfoBlock.body.mDistance;

    double gSubLonRad = sublon * M_PI / 180.;
    double gSubLatRad = sublat * M_PI / 180.;

    const double r1 = radius(aLatitude);

    // cos(A) = (b^2 + c^2 - a^2) / 2 * b * c  /*  cosine formula */
    // A = arccos((b^2 + c^2 - a^2) / 2 * b * c)
    // b == c == 1 -> A = arccos(1 - ((a^2) / 2))
    //
    // distance = |p(r, theta, phi) - p(r, 0, 0)| = |p(x, y, z) - p(r, 0, 0)|
    // z = r * sin(phi)
    // x = r * cos(phi) * cos(theta)
    // y = r * cos(phi) * sin(theta)
    // distance^2 = (r - x)^2 + y^2 + z^2;
    // r = 1 -> b = 1, c = 1, distance^2  = a^2
    // distance^2 = 1 - 2cos(theta)cos(phi) + cos^2(theta)cos^2(phi) +
    //              sin^2(theta)cos^2(phi) + sin^2(phi)
    //            = 2 - 2cos(theta)cos(phi)
    // i.e. A = arccos(cos(theta)cos(phi))
    //
    // Another solution:
    // cos(A) = x / r
    //        = cos(theta)cos(phi)
    // i.e. A = arccos(cos(theta)cos(phi))
    //
    // tan(zenith) = parpendicular / foot
    //             = d * sin(A) / {(d * cosA) - r}
    double cosA = cos(aLongitude - gSubLonRad) * cos(aLatitude - gSubLatRad);
    double sinA = sqrt(1. - pow(cosA, 2));
    double foot = (d * cosA) - r1;
    if (0. == foot) {
      return M_PI / 2.;
    }
    return atan(d * sinA / foot);
  }
};

struct HimawariStandardDataBand {
  HimawariStandardDataBand() {}
  std::vector<HimawariStandardDataSegment*> mSegments;

  struct LineCompare {
    LineCompare() {}
    bool operator() (const HimawariStandardDataSegment* aItem,
                     const int32_t& aInt) const {
      return aItem->mSegmentInfoBlock.body.mSegmentOffsetY < aInt;
    }

    bool operator() (const int32_t& aInt,
                     const HimawariStandardDataSegment* aItem) const {
      return aInt < aItem->mSegmentInfoBlock.body.mSegmentOffsetY;
    }
  };

  struct SegmentCompare {
    SegmentCompare() {}
    bool operator() (const HimawariStandardDataSegment* aItemL,
                     const HimawariStandardDataSegment* aItemR) const {
      return aItemL->mSegmentInfoBlock.body.mSegmentOffsetY <
             aItemR->mSegmentInfoBlock.body.mSegmentOffsetY;
    }
  };

  inline uint16_t countAt(double aLongitude, double aLatitude) const {
    if (!hasData()) {
      return 0;
    }
    double dataX = 0.;
    double dataY = 0.;
    mSegments[0]->mProjectionInfoBlock.body.lonlatToXY(aLongitude, aLatitude,
                                                       dataX, dataY);
    mSegments[0]->mPositionInfoBlock.rotate(dataX, dataY, true);
    mSegments[0]->mPositionInfoBlock.backwardShift(dataX, dataY);

    int32_t intX = int32_t(dataX + 0.5);
    int32_t intY = int32_t(dataY + 0.5);
    static const LineCompare comp;
    auto itr = upper_bound(mSegments.begin(), mSegments.end(), intY, comp);
    if (mSegments.begin() == itr) {
      return 0;
    }
    else {
      itr--;
    }
    return (*itr)->countAt(intX, intY);
  }

  bool hasData() const {
    return !mSegments.empty();
  }

  double radiationAt(double aLongitude, double aLatitude) const {
    uint16_t count = countAt(aLongitude, aLatitude);

    if (count & 0x8000) {
      // 0xFFFF : Error
      // 0xFFFE : Out of scope
      return 0.;
    }

    const CalibrationInfoBlockBody& calibration =
      mSegments[0]->mCalibrationInfoBlock.body;
    count &= (0xFFFF >> (16 - calibration.mValidBits));

    return (double(count) * calibration.mCoefficient) + calibration.mConstant;
  }

  double temperatureAt(double aLongitude, double aLatitude) const {
    const double rad = radiationAt(aLongitude, aLatitude) * 1000000.;
    const CalibrationInfoBlockBody& calibration =
      mSegments[0]->mCalibrationInfoBlock.body;
    const CalibrationInfoBlockBodyInfrared& infrared = calibration.u.infrared;

    const double& h = infrared.mH;
    const double& c = infrared.mC;
    const double& k = infrared.mK;
    const double l = calibration.mWaveLength / 1000000.;

    double tmp = log(1. + (2. * h * pow(c, 2) / (pow(l, 5) * rad)));
    return (h * c) / (k * l * tmp) ;
  }

  double brightnessTemperatureAt(double aLongitude, double aLatitude) const {
    const double t = temperatureAt(aLongitude, aLatitude);
    const CalibrationInfoBlockBody& calibration =
      mSegments[0]->mCalibrationInfoBlock.body;
    const CalibrationInfoBlockBodyInfrared& infrared = calibration.u.infrared;

    const double& c0 = infrared.mC0;
    const double& c1 = infrared.mC1;
    const double& c2 = infrared.mC2;

    return c0 + (c1 * t) + (c2 * pow(t, 2));
  }

  inline void sort() {
    static const SegmentCompare comp;
    std::sort(mSegments.begin(), mSegments.end(), comp);
  }
};

struct HimawariStandardData {

  std::vector<HimawariStandardDataSegment> mSegments;
  HimawariStandardDataBand mBands[16];

  HimawariStandardData() {}
  HimawariStandardData(uint32_t aHint) {
    mSegments.reserve(aHint);
  }

  void append(const char* aFileName) {
    mSegments.emplace_back();
    HimawariStandardDataSegment& segment(mSegments.back());
    segment.readFile(aFileName);
  }

  void sort() {
    for (HimawariStandardDataSegment& segment: mSegments) {
      int32_t bandID = segment.band();
      assert(1 <= bandID && bandID <= 16);
      mBands[bandID - 1].mSegments.push_back(&segment);
    }

    for (HimawariStandardDataBand& band: mBands) {
      band.sort();
    }
  }

  void brightnessTemperaturesAt(double longitude, double latitude,
                                double& aTb11, double& aTb12) const {
    // aTb11: The brightness temperture of wavelength 11 [micro meter]
    // aTb12: That of 12 [um]
    aTb11 = (mBands[13].hasData())? 
      mBands[13].brightnessTemperatureAt(longitude, latitude) : 0.;
    aTb12 = (mBands[14].hasData())? 
      mBands[14].brightnessTemperatureAt(longitude, latitude) : 0.;
  }
};
} // hsd2tms
#endif
