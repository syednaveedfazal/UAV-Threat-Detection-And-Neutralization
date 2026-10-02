using System;
using System.IO;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using NUnit.Framework;
using UnityEngine;

namespace Guardian.Sim.Tests
{
    /// <summary>
    /// Gimbal camera geometry and projection against docs/frame_test_vectors.json
    /// (gimbal_camera), generated independently in Python - the same values the
    /// ROS bridge is tested with. The projection test goes through a real Unity
    /// Camera, so it also checks the axis swap, LookRotation handedness, the
    /// vertical-FOV conversion and the pixel-centre convention.
    /// </summary>
    public class GimbalTests
    {
        static JArray _cases;
        static JArray Cases => _cases ??= (JArray)JsonConvert.DeserializeObject<JObject>(
            File.ReadAllText(Path.Combine(FrameAndSunTests.RepoRoot(), "docs", "frame_test_vectors.json")),
            SceneManifest.Settings)["gimbal_camera"];

        static double[] D(JToken t) => t.ToObject<double[]>();

        static void Near(double[] got, JToken exp, double tol, string what)
        {
            for (int i = 0; i < 3; i++) Assert.AreEqual((double)exp[i], got[i], tol, $"{what}[{i}]");
        }

        [Test]
        public void GeometryMatchesReference()
        {
            Assert.That(Cases.Count, Is.GreaterThanOrEqualTo(4));
            foreach (var c in Cases)
            {
                var g = GimbalMath.Compute(D(c["drone_pos_enu"]), D(c["drone_quat_xyzw"]), D(c["mount_flu"]),
                                           (double)c["gimbal_pitch_deg"]);
                string id = c["drone_rpy_deg"].ToString(Formatting.None);
                Near(g.Position, c["camera_pos_enu"], 1e-6, id + " camera position");
                Near(g.Right, c["axis_right_enu"], 1e-6, id + " right");
                Near(g.Down, c["axis_down_enu"], 1e-6, id + " down");
                Near(g.Forward, c["axis_forward_enu"], 1e-6, id + " forward");
            }
        }

        [Test]
        public void IntrinsicsMatchReference()
        {
            var c = Cases[0];
            var k = Intrinsics.FromHfov((int)c["width"], (int)c["height"], (double)c["hfov_deg"]);
            Assert.AreEqual((double)c["fx"], k.Fx, 1e-6);
            Assert.AreEqual((double)c["cx"], k.Cx, 1e-9);
            Assert.AreEqual((double)c["cy"], k.Cy, 1e-9);
        }

        /// <summary>World points through a Unity Camera must land on the reference pixels.</summary>
        [Test]
        public void UnityCameraProjectsLikeReference()
        {
            double tol = 0.05, worst = 0;   // px; float precision gives ~0.01
            var go = new GameObject("gimbal test cam") { hideFlags = HideFlags.HideAndDontSave };
            RenderTexture rt = null;
            try
            {
                var cam = go.AddComponent<Camera>();
                cam.enabled = false;
                foreach (var c in Cases)
                {
                    int w = (int)c["width"], h = (int)c["height"];
                    var k = Intrinsics.FromHfov(w, h, (double)c["hfov_deg"]);
                    if (rt == null) { rt = new RenderTexture(w, h, 24); cam.targetTexture = rt; }
                    GimbalMath.Configure(cam, k);
                    var g = GimbalMath.Compute(D(c["drone_pos_enu"]), D(c["drone_quat_xyzw"]), D(c["mount_flu"]),
                                               (double)c["gimbal_pitch_deg"]);
                    var (p, r) = GimbalMath.ToUnity(g);
                    go.transform.SetPositionAndRotation(p, r);
                    Assert.AreEqual(w, cam.pixelWidth); Assert.AreEqual(h, cam.pixelHeight);

                    var pts = (JArray)c["world_points_enu"];
                    var pix = (JArray)c["pixels_uv"];
                    Assert.That(pts.Count, Is.GreaterThanOrEqualTo(3));
                    for (int i = 0; i < pts.Count; i++)
                    {
                        var e = D(pts[i]);
                        var sp = cam.WorldToScreenPoint(Frames.EnuToUnity(e[0], e[1], e[2]));
                        // Unity screen: origin bottom-left, pixel i spans [i, i+1).
                        // ROS: origin top-left, pixel centres at integers.
                        double u = sp.x - 0.5, v = (h - sp.y) - 0.5;
                        double du = u - (double)pix[i][0], dv = v - (double)pix[i][1];
                        worst = Math.Max(worst, Math.Max(Math.Abs(du), Math.Abs(dv)));
                        Assert.Less(Math.Abs(du), tol, $"{c["drone_rpy_deg"].ToString(Formatting.None)} point {i} u");
                        Assert.Less(Math.Abs(dv), tol, $"{c["drone_rpy_deg"].ToString(Formatting.None)} point {i} v");
                        Assert.Greater(sp.z, 0, "point must be in front of the camera");
                    }
                }
                Debug.Log($"[Guardian] gimbal projection: worst error {worst:F4} px");
            }
            finally
            {
                if (rt != null) { rt.Release(); UnityEngine.Object.DestroyImmediate(rt); }
                UnityEngine.Object.DestroyImmediate(go);
            }
        }

        /// <summary>BuildFrame must produce exactly the layout docs/UNITY_BRIDGE.md specifies.</summary>
        [Test]
        public void FrameMessageLayout()
        {
            var g = new GimbalPose { Position = new[] { 1.0, 2, 3 }, Right = new[] { 4.0, 5, 6 },
                                     Down = new[] { 7.0, 8, 9 }, Forward = new[] { 10.0, 11, 12 } };
            var rgba = new byte[2 * 3 * 4];
            for (int i = 0; i < rgba.Length; i++) rgba[i] = (byte)i;
            var msg = BridgeClient.BuildFrame(42.5, g, 2, 3, rgba);
            Assert.AreEqual(12 + 116 + 2 * 3 * 3, msg.Length);
            Assert.AreEqual("GRD1", System.Text.Encoding.ASCII.GetString(msg, 0, 4));
            Assert.AreEqual(3, msg[4]);
            Assert.AreEqual(116 + 18, BitConverter.ToUInt32(msg, 8));
            Assert.AreEqual(42.5, BitConverter.ToDouble(msg, 12));
            for (int i = 0; i < 12; i++) Assert.AreEqual(i + 1.0, BitConverter.ToDouble(msg, 20 + 8 * i));
            Assert.AreEqual(2u, BitConverter.ToUInt32(msg, 12 + 104));
            Assert.AreEqual(3u, BitConverter.ToUInt32(msg, 12 + 108));
            Assert.AreEqual(1u, BitConverter.ToUInt32(msg, 12 + 112), "encoding: bottom row first");
            Assert.AreEqual(new byte[] { 0, 1, 2, 4, 5, 6 }, new ArraySegment<byte>(msg, 128, 6).ToArray(), "RGBA -> RGB");
        }
    }
}
