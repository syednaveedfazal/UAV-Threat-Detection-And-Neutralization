using System;
using System.Collections.Generic;
using UnityEngine;

namespace Guardian.Sim
{
    /// <summary>
    /// Moves a person along the manifest's timed waypoints, driven by SimClock.
    /// Same waypoints Gazebo's actor script uses, so both engines agree on where
    /// each person is at a given simulation time.
    /// </summary>
    public class ActorPath : MonoBehaviour
    {
        public string actorName;
        public bool loop = true;
        List<Waypoint> _waypoints;

        public void Init(Actor actor)
        {
            actorName = actor.Name;
            loop = actor.Loop;
            _waypoints = actor.Waypoints;
            Apply(SimClock.Now);
        }

        void Update()
        {
            if (_waypoints != null) Apply(SimClock.Now);
        }

        void Apply(double t)
        {
            var (x, y, z, yaw) = Evaluate(_waypoints, t, loop);
            transform.SetPositionAndRotation(Frames.EnuToUnity(x, y, z), Frames.YawZForward(yaw));
        }

        /// <summary>
        /// Position (ENU, feet on the ground) and heading at time t. Linear between
        /// waypoints; a looped path restarts at t = 0 after its last waypoint.
        /// </summary>
        public static (double x, double y, double z, double yaw) Evaluate(
            IReadOnlyList<Waypoint> w, double t, bool loop)
        {
            if (w == null || w.Count == 0) throw new ArgumentException("no waypoints");
            double end = w[w.Count - 1].T;
            if (loop && end > 0) t = ((t % end) + end) % end;
            if (t <= w[0].T) return (w[0].X, w[0].Y, w[0].ZGround, w[0].Yaw);
            if (t >= end) { var l = w[w.Count - 1]; return (l.X, l.Y, l.ZGround, l.Yaw); }

            int i = 0;
            while (i < w.Count - 2 && w[i + 1].T < t) i++;
            var a = w[i];
            var b = w[i + 1];
            double f = b.T > a.T ? (t - a.T) / (b.T - a.T) : 0;
            double dyaw = Math.IEEERemainder(b.Yaw - a.Yaw, 2 * Math.PI);   // shortest turn
            return (a.X + (b.X - a.X) * f, a.Y + (b.Y - a.Y) * f,
                    a.ZGround + (b.ZGround - a.ZGround) * f, a.Yaw + dyaw * f);
        }
    }
}
