using System;
using Newtonsoft.Json;
using UnityEngine;

namespace Guardian.Sim
{
    /// <summary>
    /// The drone in Unity: follows Gazebo's ground-truth pose from the bridge,
    /// carries the stabilised gimbal camera, and makes Gazebo's sim time the
    /// clock for everything else (SimClock), so the walking people are where
    /// Gazebo has them in the very frame the camera renders.
    ///
    /// Camera spec (stream size, HFOV, rate, mount, gimbal pitch) comes from the
    /// scene manifest - the same numbers the ROS side publishes as CameraInfo.
    /// Command line (player): --bridge-host H --bridge-port P.
    /// </summary>
    [DefaultExecutionOrder(-50)]          // before ActorPath.Update, which reads SimClock
    public class DroneRig : MonoBehaviour
    {
        /// <summary>The drone body lives on this layer, which the gimbal camera does not render.</summary>
        public const int DroneLayer = 31;

        public string host = "127.0.0.1";
        public int port = 5700;
        [Tooltip("Main Camera follows the drone from behind")]
        public bool chaseMainCamera = true;
        public bool showFeed = true;

        public SiteLoader site;
        public GimbalCamera Gimbal { get; private set; }
        public BridgeClient Bridge { get; private set; }

        Transform _body;
        double[] _mount = { 0.10, 0, -0.08 };
        double _pitchDeg = -35;
        long _lastSeq = -1;
        double _simTime = double.NaN;
        PoseSample _pose;
        string _siteName = "";
        Vector3 _chaseVel;

        void Start()
        {
            Application.runInBackground = true;   // keep streaming when the window loses focus
            ReadArgs();
            if (site == null) site = FindAnyObjectByType<SiteLoader>();

            var k = Intrinsics.FromHfov(960, 540, 82);
            float rate = 15;
            try
            {
                var m = site != null && site.Manifest != null ? site.Manifest
                      : SceneManifest.Load(SiteLoader.ResolveManifestPath(site != null ? site.manifestPath : ""));
                _siteName = m.Site;
                var c = m.Camera;
                if (c != null)
                {
                    k = Intrinsics.FromHfov((int)c["stream_px"][0], (int)c["stream_px"][1], (double)c["hfov_deg"]);
                    rate = (float)c["rate_hz"];
                    _mount = c["mount_xyz"].ToObject<double[]>();
                    _pitchDeg = (double)c["gimbal"]["pitch_deg"];
                }
            }
            catch (Exception e) { Debug.LogWarning("[Guardian] camera spec from manifest failed, using defaults: " + e.Message); }

            _body = Placeholders.Drone(transform, DroneLayer).transform;
            _body.gameObject.SetActive(false);   // until the first pose arrives

            var camGo = new GameObject("Gimbal Camera");
            camGo.transform.SetParent(transform, false);
            camGo.AddComponent<Camera>();
            Gimbal = camGo.AddComponent<GimbalCamera>();

            var hello = JsonConvert.SerializeObject(new
            {
                protocol = 1, site = _siteName,
                camera = new { width = k.Width, height = k.Height, fx = k.Fx, fy = k.Fy, cx = k.Cx, cy = k.Cy,
                               hfov_deg = k.HfovDeg, distortion_model = "plumb_bob",
                               d = new double[5] },   // rendered image is an ideal pinhole (lens model: Phase 4)
            });
            Bridge = new BridgeClient(host, port, () => hello);
            Gimbal.Init(k, rate, Bridge, ~(1 << DroneLayer));
            Debug.Log($"[Guardian] drone camera {k.Width}x{k.Height} hfov {k.HfovDeg} deg @ {rate} Hz, " +
                      $"gimbal pitch {_pitchDeg} deg; bridge {host}:{port}");
        }

        void ReadArgs()
        {
            var a = Environment.GetCommandLineArgs();
            for (int i = 0; i + 1 < a.Length; i++)
            {
                if (a[i] == "--bridge-host") host = a[i + 1];
                if (a[i] == "--bridge-port" && int.TryParse(a[i + 1], out var p)) port = p;
            }
        }

        void Update()
        {
            if (Bridge == null || !Bridge.TryGetPose(out var pose) || pose.Seq == _lastSeq) return;
            _lastSeq = pose.Seq;
            _pose = pose;
            _simTime = pose.SimTime;
            SimClock.Source ??= () => _simTime;

            _body.gameObject.SetActive(true);
            _body.SetPositionAndRotation(
                Frames.EnuToUnity(pose.Position[0], pose.Position[1], pose.Position[2]),
                Frames.RosToUnity(pose.Rotation[0], pose.Rotation[1], pose.Rotation[2], pose.Rotation[3]));

            // Stream only a finished site: frames of a half-loaded scene would be wrong data.
            if (site == null || site.IsLoaded)
                Gimbal.Place(GimbalMath.Compute(pose.Position, pose.Rotation, _mount, _pitchDeg), pose.SimTime);
        }

        void LateUpdate()
        {
            if (!chaseMainCamera || _lastSeq < 0) return;
            var cam = Camera.main;
            if (cam == null) return;
            var fwd = _body.right; fwd.y = 0;                 // body +X = forward
            if (fwd.sqrMagnitude < 1e-4f) fwd = Vector3.forward;
            fwd.Normalize();
            var want = _body.position - fwd * 6f + Vector3.up * 3f;
            cam.transform.position = Vector3.SmoothDamp(cam.transform.position, want, ref _chaseVel, 0.3f);
            cam.transform.rotation = Quaternion.LookRotation(_body.position + fwd * 4f - cam.transform.position, Vector3.up);
        }

        void OnGUI()
        {
            if (!showFeed || Gimbal == null || Gimbal.Target == null) return;
            float w = Mathf.Min(480, Screen.width * 0.4f), h = w * Gimbal.Target.height / Gimbal.Target.width;
            var r = new Rect(Screen.width - w - 10, Screen.height - h - 10, w, h);
            GUI.DrawTexture(r, Gimbal.Target, ScaleMode.ScaleToFit, false);
            string pos = _lastSeq < 0 ? "no pose yet"
                : $"sim {_simTime:F2} s   drone E {_pose.Position[0]:F1} N {_pose.Position[1]:F1} U {_pose.Position[2]:F1} m";
            GUI.Label(new Rect(r.x, r.y - 44, w, 44),
                $"<b>Gimbal camera</b>  {Bridge?.Status}\n{pos}   sent {Bridge?.FramesSent} dropped {Bridge?.FramesDropped}",
                new GUIStyle(GUI.skin.label) { richText = true, normal = { textColor = Color.white } });
        }

        void OnDestroy()
        {
            SimClock.Source = null;
            Bridge?.Dispose();
        }
    }
}
