/**
 * Static methods for handling of Coordinate System Transformations.
 */
var Coordinates = {};

/**
 * Transformation from J2000 to ECEF coordinates.
 * 
 * @param {*} r
 *      Position vector in J2000 frame.. 
 * @param {*} JD 
 *      Julian date.
 * @param {*} JT 
 *      Julian time.
 * @returns Position vector in ECEF frame.
 */
Coordinates.J2000ToECEF = function(r, JD, JT)
{
    let MJD = JT - 2400000.5;
    // IAU 1976 Precession Model
    // (ESA GNSS Data Processing Volume 1 - A2.5.1)
    let T = (MJD - 0.5)/36525.0;
    let z    = 0.6406161388 * T + 3.0407777777e-04 * T*T + 5.0563888888e-06 *T*T*T;
    let nu   = 0.5567530277 * T - 1.1851388888e-04 * T*T - 1.1620277777e-05 *T*T*T;
    let zeta = 0.6406161388 * T + 8.3855555555e-05 * T*T + 4.9994444444e-06 *T*T*T;
    let rCEP = MathUtils.rotZ(MathUtils.rotX(MathUtils.rotZ(r, -90.0 + zeta), -nu), 90.0 + z);

    // We do not take Nutation nor polar movement into account. 

    // Rotation.
    return MathUtils.rotZ(rCEP, -TimeConversions.computeSiderealTime(0, JD, JT));
}
