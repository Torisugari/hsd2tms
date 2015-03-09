dset    ^CTOP_LUT.mt2.v20.grd
title   CTOP LUT for MTSAT-2
options little_endian
undef   -1e3
*
xdef   131 linear 180.0  1.0  ** 11-um brightness temperature (TB)
ydef   131 linear  -4.0  0.1  ** TB difference (11um-12um)
zdef     6 linear   7.5 15.0  ** Satellite zenith angle [degree]
tdef     1 linear jan0001 1yr
*
vars 7
rcld  6 99 Rate of cloudy samples [%]
tope  6 99 Cloud top height estimate [km]
topd  6 99 Cloud top height stdev [km]
tops  6 99 Cloud top height stderr [km]
taue  6 99 Cloud visible optical thickness estimate [a.u.]
taud  6 99 Cloud visible optical thickness stdev [a.u.]
taus  6 99 Cloud visible optical thickness stderr [a.u.]
endvars
