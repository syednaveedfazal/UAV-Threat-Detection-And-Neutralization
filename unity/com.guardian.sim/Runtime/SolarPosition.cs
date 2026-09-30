using System;
using UnityEngine;

namespace Guardian.Sim
{
    /// <summary>
    /// Sun position from place and time (NOAA general solar position equations,
    /// plus atmospheric refraction). Accurate to a few tenths of a degree - enough
    /// for correct shadow directions; tested against NREL SPA (pvlib) values in
    /// docs/frame_test_vectors.json.
    /// </summary>
    public static class SolarPosition
    {
        /// <summary>Elevation (deg above horizon, refraction-corrected) and azimuth
        /// (deg clockwise from North) for a UTC instant.</summary>
        public static (double elevationDeg, double azimuthDeg) Compute(DateTime utc, double latDeg, double lonDeg)
        {
            if (utc.Kind == DateTimeKind.Local) utc = utc.ToUniversalTime();
            double hour = utc.Hour + utc.Minute / 60.0 + utc.Second / 3600.0;
            int daysInYear = DateTime.IsLeapYear(utc.Year) ? 366 : 365;
            double g = 2 * Math.PI / daysInYear * (utc.DayOfYear - 1 + (hour - 12) / 24);

            double eqTime = 229.18 * (0.000075 + 0.001868 * Math.Cos(g) - 0.032077 * Math.Sin(g)
                                      - 0.014615 * Math.Cos(2 * g) - 0.040849 * Math.Sin(2 * g));
            double decl = 0.006918 - 0.399912 * Math.Cos(g) + 0.070257 * Math.Sin(g)
                          - 0.006758 * Math.Cos(2 * g) + 0.000907 * Math.Sin(2 * g)
                          - 0.002697 * Math.Cos(3 * g) + 0.00148 * Math.Sin(3 * g);

            double trueSolarMin = hour * 60 + eqTime + 4 * lonDeg;
            double ha = Deg2Rad(trueSolarMin / 4 - 180);
            double lat = Deg2Rad(latDeg);

            double cosZen = Math.Sin(lat) * Math.Sin(decl) + Math.Cos(lat) * Math.Cos(decl) * Math.Cos(ha);
            double zen = Math.Acos(Math.Clamp(cosZen, -1, 1));
            double elev = 90 - Rad2Deg(zen);

            double az = Rad2Deg(Math.Atan2(Math.Sin(ha),
                                           Math.Cos(ha) * Math.Sin(lat) - Math.Tan(decl) * Math.Cos(lat))) + 180;
            az = (az % 360 + 360) % 360;

            // Refraction lifts the apparent sun near the horizon (Bennett's formula).
            if (elev > -1)
                elev += 1.02 / Math.Tan(Deg2Rad(elev + 10.3 / (elev + 5.11))) / 60.0;
            return (elev, az);
        }

        /// <summary>Local time string + IANA timezone (e.g. "Europe/Berlin") to UTC.</summary>
        public static DateTime LocalToUtc(string localIso, string timezone)
        {
            var local = DateTime.SpecifyKind(DateTime.Parse(localIso,
                System.Globalization.CultureInfo.InvariantCulture), DateTimeKind.Unspecified);
            var tz = TimeZoneInfo.FindSystemTimeZoneById(timezone);
            return TimeZoneInfo.ConvertTimeToUtc(local, tz);
        }

        /// <summary>Unity direction pointing FROM the scene TOWARDS the sun.</summary>
        public static Vector3 SunDirectionUnity(double elevationDeg, double azimuthDeg)
        {
            double el = Deg2Rad(elevationDeg), az = Deg2Rad(azimuthDeg);
            // ENU: east = sin(az) cos(el), north = cos(az) cos(el), up = sin(el)
            return Frames.EnuToUnity(Math.Sin(az) * Math.Cos(el), Math.Cos(az) * Math.Cos(el), Math.Sin(el));
        }

        /// <summary>Rotation for a directional light (its forward points away from the sun).</summary>
        public static Quaternion LightRotation(double elevationDeg, double azimuthDeg) =>
            Quaternion.LookRotation(-SunDirectionUnity(elevationDeg, azimuthDeg), Vector3.up);

        static double Deg2Rad(double d) => d * Math.PI / 180;
        static double Rad2Deg(double r) => r * 180 / Math.PI;
    }
}
