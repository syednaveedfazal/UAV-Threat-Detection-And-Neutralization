using System;
using UnityEngine;

namespace Guardian.Sim
{
    /// <summary>
    /// Simulation time in seconds. Until the ROS bridge exists (Phase 3) it runs on
    /// Unity's own clock; the bridge then points Source at Gazebo's /clock so
    /// people walk exactly where Gazebo has them.
    /// </summary>
    public static class SimClock
    {
        public static Func<double> Source;

        public static double Now => Source != null ? Source() : Time.timeAsDouble;
    }
}
