using System;
using UnityEngine;

namespace Guardian.Sim
{
    /// <summary>Camera pose in ROS/ENU: position and the optical-frame axes (x right, y down, z forward).</summary>
    public struct GimbalPose
    {
        public double[] Position;   // ENU, m
        public double[] Right;      // ENU unit vectors
        public double[] Down;
        public double[] Forward;
    }

    /// <summary>Pinhole intrinsics, ROS convention (pixel centres at integer coordinates).</summary>
    [Serializable]
    public struct Intrinsics
    {
        public int Width, Height;
        public double Fx, Fy, Cx, Cy, HfovDeg;

        public static Intrinsics FromHfov(int width, int height, double hfovDeg)
        {
            double fx = width / 2.0 / Math.Tan(hfovDeg * Math.PI / 360);
            return new Intrinsics { Width = width, Height = height, Fx = fx, Fy = fx,
                                    Cx = (width - 1) / 2.0, Cy = (height - 1) / 2.0, HfovDeg = hfovDeg };
        }

        /// <summary>Unity's Camera.fieldOfView is vertical.</summary>
        public float VerticalFovDeg => (float)(2 * Math.Atan(Height / 2.0 / Fy) * 180 / Math.PI);
    }

    /// <summary>
    /// Stabilised gimbal camera geometry - the definition in docs/UNITY_BRIDGE.md,
    /// identical to unity_bridge_protocol.gimbal_camera on the ROS side; both are
    /// tested against docs/frame_test_vectors.json (gimbal_camera).
    /// Double precision in ENU; converted to Unity only in ToUnity.
    /// </summary>
    public static class GimbalMath
    {
        /// <param name="pos">drone position ENU</param>
        /// <param name="q">drone orientation, ROS quaternion (x, y, z, w)</param>
        /// <param name="mountFlu">camera mount in the body frame (forward, left, up), m</param>
        /// <param name="pitchDeg">gimbal pitch, negative = looking down</param>
        public static GimbalPose Compute(double[] pos, double[] q, double[] mountFlu, double pitchDeg)
        {
            var r = QuatToMatrix(q);
            var cam = new double[3];
            for (int i = 0; i < 3; i++)
                cam[i] = pos[i] + r[i, 0] * mountFlu[0] + r[i, 1] * mountFlu[1] + r[i, 2] * mountFlu[2];
            // heading = body x-axis projected on the ground; roll and pitch are what the gimbal removes
            double psi = Math.Atan2(r[1, 0], r[0, 0]);
            double p = pitchDeg * Math.PI / 180;
            var fwd = new[] { Math.Cos(psi) * Math.Cos(p), Math.Sin(psi) * Math.Cos(p), Math.Sin(p) };
            var right = new[] { Math.Sin(psi), -Math.Cos(psi), 0.0 };
            var down = new[] { fwd[1] * right[2] - fwd[2] * right[1],
                               fwd[2] * right[0] - fwd[0] * right[2],
                               fwd[0] * right[1] - fwd[1] * right[0] };
            return new GimbalPose { Position = cam, Right = right, Down = down, Forward = fwd };
        }

        /// <summary>
        /// Unity transform for a camera with this pose. A Unity camera looks along
        /// its +Z with +Y up on screen, so forward -> +Z and (-down) -> +Y. The
        /// ENU->Unity axis swap turns the right-handed optical triad into Unity's
        /// left-handed one, so screen-right comes out as the optical x axis.
        /// </summary>
        public static (Vector3 position, Quaternion rotation) ToUnity(GimbalPose g)
        {
            var f = Frames.EnuToUnity(g.Forward[0], g.Forward[1], g.Forward[2]);
            var up = Frames.EnuToUnity(-g.Down[0], -g.Down[1], -g.Down[2]);
            return (Frames.EnuToUnity(g.Position[0], g.Position[1], g.Position[2]),
                    Quaternion.LookRotation(f, up));
        }

        /// <summary>Symmetric pinhole on a Unity camera; the render target sets the pixel size.</summary>
        public static void Configure(Camera cam, Intrinsics k)
        {
            cam.usePhysicalProperties = false;
            cam.fieldOfView = k.VerticalFovDeg;
            cam.aspect = (float)k.Width / k.Height;
        }

        public static double[,] QuatToMatrix(double[] q)
        {
            double x = q[0], y = q[1], z = q[2], w = q[3];
            double n = Math.Sqrt(x * x + y * y + z * z + w * w);
            x /= n; y /= n; z /= n; w /= n;
            return new[,]
            {
                { 1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w) },
                { 2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w) },
                { 2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y) },
            };
        }
    }
}
