using System;
using System.IO;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using NUnit.Framework;
using UnityEngine;

namespace Guardian.Sim.Tests
{
    /// <summary>
    /// Frame and sun math against docs/frame_test_vectors.json, which is generated
    /// independently in Python (rotation matrices, pvlib) - not by this code.
    /// </summary>
    public class FrameAndSunTests
    {
        static JObject _v;

        internal static string RepoRoot()
        {
            var pkg = UnityEditor.PackageManager.PackageInfo.FindForAssembly(typeof(FrameAndSunTests).Assembly);
            return Path.GetFullPath(Path.Combine(pkg.resolvedPath, "..", ".."));
        }

        // DateParseHandling.None: otherwise "…Z" timestamps are silently turned into
        // local-time DateTimes and every UTC comparison is off by the machine's offset.
        static JObject V => _v ??= JsonConvert.DeserializeObject<JObject>(
            File.ReadAllText(Path.Combine(RepoRoot(), "docs", "frame_test_vectors.json")),
            SceneManifest.Settings);

        static void Near(Vector3 got, JToken exp, double tol, string what)
        {
            Assert.AreEqual((double)exp[0], got.x, tol, what + " x");
            Assert.AreEqual((double)exp[1], got.y, tol, what + " y");
            Assert.AreEqual((double)exp[2], got.z, tol, what + " z");
        }

        [Test]
        public void Points()
        {
            foreach (var p in V["points"])
            {
                var e = p["enu"];
                var u = Frames.EnuToUnity((double)e[0], (double)e[1], (double)e[2]);
                Near(u, p["unity"], 1e-5, (string)p["note"]);
                Near(Frames.UnityToEnu(u), e, 1e-5, "round trip");
            }
        }

        [Test]
        public void YawOfXAlignedAndForwardFacingObjects()
        {
            foreach (var y in V["yaw"])
            {
                double yaw = (double)y["yaw_enu_deg"] * Math.PI / 180;
                Near(Frames.YawXAligned(yaw) * Vector3.right, y["direction_unity"], 1e-5,
                     $"x-aligned yaw {y["yaw_enu_deg"]}");
                Near(Frames.YawZForward(yaw) * Vector3.forward, y["direction_unity"], 1e-5,
                     $"z-forward yaw {y["yaw_enu_deg"]}");
            }
        }

        [Test]
        public void RosQuaternionsToUnity()
        {
            foreach (var r in V["rotations"])
            {
                var q = r["ros_quat_xyzw"];
                var u = Frames.RosToUnity((double)q[0], (double)q[1], (double)q[2], (double)q[3]);
                var e = r["unity_quat_xyzw"];
                var expected = new Quaternion((float)e[0], (float)e[1], (float)e[2], (float)e[3]);
                // q and -q are the same rotation
                Assert.Greater(Mathf.Abs(Quaternion.Dot(u, expected)), 1 - 1e-5, $"rpy {r["ros_rpy_deg"]}");
                Near(u * Vector3.right, r["ros_x_axis_in_unity"], 1e-5, $"x axis, rpy {r["ros_rpy_deg"]}");
            }
        }

        [Test]
        public void SunMatchesPvlib()
        {
            double tol = (double)V["tolerance"]["sun_deg"];
            foreach (var s in V["sun"])
            {
                var utc = SolarPosition.LocalToUtc((string)s["local_time"], (string)s["timezone"]);
                var expectedUtc = DateTime.Parse((string)s["utc"], System.Globalization.CultureInfo.InvariantCulture,
                    System.Globalization.DateTimeStyles.AdjustToUniversal | System.Globalization.DateTimeStyles.AssumeUniversal);
                Assert.AreEqual(expectedUtc, utc, "timezone conversion");
                var (el, az) = SolarPosition.Compute(utc, (double)s["lat"], (double)s["lon"]);
                Assert.AreEqual((double)s["elevation_deg"], el, tol, $"elevation {s["local_time"]}");
                Assert.AreEqual((double)s["azimuth_deg_from_north_cw"], az, tol, $"azimuth {s["local_time"]}");
            }
        }

        [Test]
        public void SunLightPointsAwayFromTheSun()
        {
            // Sun due south, 30 deg up: light must travel north and downwards.
            var fwd = SolarPosition.LightRotation(30, 180) * Vector3.forward;
            Assert.Greater(fwd.z, 0.8f, "towards north (+Z)");
            Assert.Less(fwd.y, -0.45f, "downwards");
        }
    }
}
