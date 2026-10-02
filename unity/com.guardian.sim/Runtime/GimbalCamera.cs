using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering;

namespace Guardian.Sim
{
    /// <summary>
    /// The drone's gimbal camera. DroneRig tells it where to be (Place); it
    /// renders at the manifest's camera rate into a render texture, reads the
    /// pixels back asynchronously and hands them to the bridge together with the
    /// sim time and camera pose it rendered with - so the ROS image, its TF and
    /// any labels always describe the same instant.
    ///
    /// The camera component stays disabled and is rendered on demand, so it
    /// costs GPU time only at the camera rate, not at the editor's frame rate.
    /// </summary>
    [RequireComponent(typeof(Camera))]
    public class GimbalCamera : MonoBehaviour
    {
        public Intrinsics intrinsics = Intrinsics.FromHfov(960, 540, 82);
        public float rateHz = 15;
        [Tooltip("Readbacks allowed in flight; more adds latency, fewer can starve the GPU")]
        public int maxInFlight = 2;

        public RenderTexture Target { get; private set; }
        public long FramesRendered { get; private set; }
        public double LastFrameSimTime { get; private set; } = double.NaN;

        Camera _cam;
        BridgeClient _bridge;
        GimbalPose _pose;
        double _poseSimTime = double.NaN;
        float _nextRender;
        int _inFlight;

        public void Init(Intrinsics k, float rate, BridgeClient bridge, int cullingMask)
        {
            intrinsics = k; rateHz = rate; _bridge = bridge;
            _cam = GetComponent<Camera>();
            _cam.enabled = false;
            _cam.nearClipPlane = 0.2f;
            _cam.farClipPlane = 1500;
            _cam.cullingMask = cullingMask;
            GimbalMath.Configure(_cam, k);
            if (Target) Target.Release();
            Target = new RenderTexture(k.Width, k.Height, 24, RenderTextureFormat.ARGB32)
                     { name = "Gimbal camera", antiAliasing = 1 };
            Target.Create();
        }

        /// <summary>Move the camera to the gimbal pose for the drone pose at simTime.</summary>
        public void Place(GimbalPose pose, double simTime)
        {
            _pose = pose;
            _poseSimTime = simTime;
            var (p, r) = GimbalMath.ToUnity(pose);
            transform.SetPositionAndRotation(p, r);
        }

        // LateUpdate: after DroneRig (Update, earlier order) has placed the camera and
        // the people (ActorPath.Update) have moved to the same sim time.
        void LateUpdate()
        {
            if (_cam == null || double.IsNaN(_poseSimTime) || Time.unscaledTime < _nextRender || _inFlight >= maxInFlight)
                return;
            _nextRender = Mathf.Max(_nextRender + 1f / rateHz, Time.unscaledTime - 0.5f / rateHz);
            Render();

            if (_bridge == null || !_bridge.Connected) return;
            // Capture this frame's pose now; the readback completes a frame or two later.
            var pose = _pose; double t = _poseSimTime; var k = intrinsics; var bridge = _bridge;
            _inFlight++;
            AsyncGPUReadback.Request(Target, 0, TextureFormat.RGBA32, req =>
            {
                _inFlight--;
                if (req.hasError) { Debug.LogWarning("[Guardian] gimbal readback failed"); return; }
                NativeArray<byte> data = req.GetData<byte>();
                bridge.SendFrame(BridgeClient.BuildFrame(t, pose, k.Width, k.Height, data.AsReadOnlySpan()));
                LastFrameSimTime = t;
            });
        }

        void Render()
        {
            var req = new RenderPipeline.StandardRequest { destination = Target };
            if (RenderPipeline.SupportsRenderRequest(_cam, req)) RenderPipeline.SubmitRenderRequest(_cam, req);
            else { _cam.targetTexture = Target; _cam.Render(); _cam.targetTexture = null; }
            FramesRendered++;
        }

        void OnDestroy()
        {
            if (Target) { Target.Release(); Destroy(Target); }
        }
    }
}
