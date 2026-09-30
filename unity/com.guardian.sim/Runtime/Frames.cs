using UnityEngine;

namespace Guardian.Sim
{
    /// <summary>
    /// The ONLY place that converts between ROS/Gazebo and Unity coordinates.
    ///
    /// ROS / Gazebo / scene manifest: ENU, right-handed - x = East, y = North, z = Up.
    /// Unity: left-handed, y up - we map x = East, y = Up, z = North.
    ///
    /// The map swaps y and z, which is a mirror (det -1). That is exactly what
    /// turns a right-handed frame into a left-handed one, so geometry is NOT
    /// mirrored in the world: East stays East, North stays North.
    /// Test values: docs/frame_test_vectors.json (generated independently in Python).
    /// </summary>
    public static class Frames
    {
        /// <summary>ENU point (metres) to Unity position.</summary>
        public static Vector3 EnuToUnity(double east, double north, double up) =>
            new Vector3((float)east, (float)up, (float)north);

        public static Vector3 EnuToUnity(Vector3 enu) => new Vector3(enu.x, enu.z, enu.y);

        public static Vector3 UnityToEnu(Vector3 u) => new Vector3(u.x, u.z, u.y);

        /// <summary>
        /// Yaw (ENU, counter-clockwise from East, radians) for objects whose length
        /// axis is their local +X (fence panels, gates). Unity's Y rotation is
        /// clockwise seen from above, hence the minus sign.
        /// </summary>
        public static Quaternion YawXAligned(double yawEnu) =>
            Quaternion.Euler(0f, -(float)(yawEnu * Mathf.Rad2Deg), 0f);

        /// <summary>Yaw for objects that face their local +Z (people, cameras, vehicles).</summary>
        public static Quaternion YawZForward(double yawEnu) =>
            Quaternion.Euler(0f, 90f - (float)(yawEnu * Mathf.Rad2Deg), 0f);

        /// <summary>
        /// ROS quaternion (x, y, z, w) to Unity. Equivalent to S R S with S = swap(y, z);
        /// verified against the matrix method in make_frame_test_vectors.py.
        /// </summary>
        public static Quaternion RosToUnity(double x, double y, double z, double w) =>
            new Quaternion(-(float)x, -(float)z, -(float)y, (float)w);

        /// <summary>
        /// The scene package's glTF files use glTF axes X = East, Y = Up, Z = -North.
        /// glTFast converts glTF (right-handed) to Unity by negating X, so the meshes
        /// arrive as (-East, Up, -North): a 180 degree turn about Up away from our
        /// convention. Parenting them under this rotation puts them back.
        /// SiteLoader verifies this at load time (south-west tile must land at -X, -Z).
        /// </summary>
        public static readonly Quaternion GltfRootCorrection = Quaternion.Euler(0f, 180f, 0f);
    }
}
