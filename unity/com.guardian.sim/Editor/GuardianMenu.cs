using System;
using System.IO;
using System.Threading.Tasks;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;

namespace Guardian.Sim.Editor
{
    /// <summary>Guardian menu: load the real site into the open scene, and capture checks.</summary>
    public static class GuardianMenu
    {
        const string PrefKey = "Guardian.ManifestPath";

        [MenuItem("Guardian/Load Site (compound)", priority = 1)]
        static void LoadCompound() => _ = Load(SiteRegion.Compound);

        [MenuItem("Guardian/Load Site (full 600 m)", priority = 2)]
        static void LoadFull() => _ = Load(SiteRegion.Full);

        [MenuItem("Guardian/Play Site (compound)", priority = 4)]
        static void PlayCompound() => PlaySite(SiteRegion.Compound);

        [MenuItem("Guardian/Play Site (full 600 m)", priority = 5)]
        static void PlayFull() => PlaySite(SiteRegion.Full);

        /// <summary>
        /// One click to watch the site: load it on Play (people walk), with the
        /// Main Camera placed high to the north-east of the compound looking at it.
        /// </summary>
        static void PlaySite(SiteRegion region)
        {
            if (EditorApplication.isPlaying) { EditorApplication.isPlaying = false; return; }
            var loader = FindLoader();
            if (loader == null)
                loader = new GameObject("Guardian Site").AddComponent<SiteLoader>();
            loader.Unload();                       // edit-mode copy; Play builds its own
            loader.region = region;
            loader.loadOnStart = true;
            loader.manifestPath = EditorPrefs.GetString(PrefKey, loader.manifestPath);

            var cam = Camera.main;
            try
            {
                var m = SceneManifest.Load(SiteLoader.ResolveManifestPath(loader.manifestPath));
                var b = m.Regions["compound"];
                double cx = (b.Min[0] + b.Max[0]) / 2, cy = (b.Min[1] + b.Max[1]) / 2;
                double back = region == SiteRegion.Compound ? 70 : 160, up = region == SiteRegion.Compound ? 55 : 120;
                if (cam != null)
                {
                    var target = Frames.EnuToUnity(cx, cy, 0);
                    var pos = Frames.EnuToUnity(cx + back, cy + back, up);
                    cam.transform.SetPositionAndRotation(pos, Quaternion.LookRotation(target - pos, Vector3.up));
                    cam.farClipPlane = 2000;
                }
            }
            catch (Exception e) { Debug.LogException(e); }
            if (cam == null) Debug.LogWarning("[Guardian] no Main Camera in the scene - use the Scene view");
            EditorApplication.isPlaying = true;
        }

        [MenuItem("Guardian/Unload Site", priority = 3)]
        static void Unload() => FindLoader()?.Unload();

        [MenuItem("Guardian/Choose Scene Manifest...", priority = 20)]
        static void Choose()
        {
            var start = Path.GetDirectoryName(SiteLoader.ResolveManifestPath(EditorPrefs.GetString(PrefKey, "")));
            var p = EditorUtility.OpenFilePanel("scene_manifest.json", start, "json");
            if (!string.IsNullOrEmpty(p)) EditorPrefs.SetString(PrefKey, p);
        }

        [MenuItem("Guardian/Capture Top-Down (alignment check)", priority = 40)]
        static void CaptureMenu()
        {
            var loader = FindLoader();
            if (loader == null || loader.Manifest == null) { Debug.LogError("[Guardian] Load a site first."); return; }
            var png = CaptureTopDown(loader, 2048);
            Debug.Log($"[Guardian] top-down capture -> {png}\n" +
                      "check: ~/.venvs/geo/bin/python scripts/check_unity_alignment.py " + png);
        }

        /// <summary>Loads (or reloads) the site into the open scene. Used by the menu and by batch mode.</summary>
        public static async Task<SiteLoader> Load(SiteRegion region)
        {
            var loader = FindLoader();
            if (loader == null)
            {
                loader = new GameObject("Guardian Site").AddComponent<SiteLoader>();
                Undo.RegisterCreatedObjectUndo(loader.gameObject, "Create Guardian Site");
            }
            loader.region = region;
            loader.manifestPath = EditorPrefs.GetString(PrefKey, loader.manifestPath);
            try
            {
                await loader.LoadAsync();
                if (SceneView.lastActiveSceneView != null)
                {
                    var b = loader.RegionBox;
                    var c = Frames.EnuToUnity((b.Min[0] + b.Max[0]) / 2, (b.Min[1] + b.Max[1]) / 2, 0);
                    SceneView.lastActiveSceneView.LookAt(c, Quaternion.Euler(45, 30, 0),
                        (float)Math.Max(b.Max[0] - b.Min[0], b.Max[1] - b.Min[1]));
                }
            }
            catch (Exception e) { Debug.LogException(e); }
            return loader;
        }

        static SiteLoader FindLoader() =>
            UnityEngine.Object.FindAnyObjectByType<SiteLoader>(FindObjectsInactive.Include);

        /// <summary>
        /// Orthographic render straight down over the loaded region, north up -
        /// the same geometry as the aerial photo, so a script can measure how far
        /// Unity's scene is shifted/rotated/mirrored from reality.
        /// Writes PNG + JSON (world bounds) next to the manifest.
        /// </summary>
        public static string CaptureTopDown(SiteLoader loader, int px)
        {
            var box = loader.RegionBox;
            double w = box.Max[0] - box.Min[0], h = box.Max[1] - box.Min[1], s = Math.Max(w, h);
            double cx = (box.Min[0] + box.Max[0]) / 2, cy = (box.Min[1] + box.Max[1]) / 2;

            var go = new GameObject("Guardian TopDown") { hideFlags = HideFlags.HideAndDontSave };
            var cam = go.AddComponent<Camera>();
            cam.orthographic = true;
            cam.orthographicSize = (float)(s / 2);
            cam.nearClipPlane = 1; cam.farClipPlane = 2000;
            cam.clearFlags = CameraClearFlags.SolidColor; cam.backgroundColor = Color.black;
            // Looking straight down with Unity +Z (north) at the top of the image.
            cam.transform.SetPositionAndRotation(Frames.EnuToUnity(cx, cy, 800), Quaternion.Euler(90, 0, 0));

            var rt = new RenderTexture(px, px, 24, RenderTextureFormat.ARGB32) { antiAliasing = 1 };
            var request = new RenderPipeline.StandardRequest { destination = rt };
            // Several frames so auto-exposure settles before the frame we keep.
            for (int i = 0; i < 12; i++)
            {
                if (RenderPipeline.SupportsRenderRequest(cam, request)) RenderPipeline.SubmitRenderRequest(cam, request);
                else { cam.targetTexture = rt; cam.Render(); cam.targetTexture = null; }
            }
            var prev = RenderTexture.active;
            RenderTexture.active = rt;
            var tex = new Texture2D(px, px, TextureFormat.RGB24, false);
            tex.ReadPixels(new Rect(0, 0, px, px), 0, 0);
            tex.Apply();
            RenderTexture.active = prev;

            var dir = loader.Manifest.Directory;
            var name = $"unity_topdown_{loader.region.ToString().ToLowerInvariant()}";
            var png = Path.Combine(dir, name + ".png");
            File.WriteAllBytes(png, tex.EncodeToPNG());
            File.WriteAllText(Path.Combine(dir, name + ".json"), JsonConvert.SerializeObject(new
            {
                world_min = new[] { cx - s / 2, cy - s / 2 },   // ENU, image bottom-left
                world_max = new[] { cx + s / 2, cy + s / 2 },   // ENU, image top-right
                px,
                m_per_px = s / px,
                orientation = "north up, east right",
                site_report = loader.LastReport,
            }, Formatting.Indented));

            UnityEngine.Object.DestroyImmediate(tex);
            rt.Release(); UnityEngine.Object.DestroyImmediate(rt);
            UnityEngine.Object.DestroyImmediate(go);
            return png;
        }

        /// <summary>
        /// Batch-mode entry: loads the compound, captures the top-down image, quits.
        ///   Unity -batchmode -projectPath unity/GuardianSim -executeMethod Guardian.Sim.Editor.GuardianMenu.BatchCapture
        /// </summary>
        public static async void BatchCapture()
        {
            int code = 0;
            try
            {
                OpenBuildScene();
                var loader = await Load(SiteRegion.Compound);
                Debug.Log("[Guardian] " + loader.LastReport);
                Debug.Log("[Guardian] capture -> " + CaptureTopDown(loader, 2048));
            }
            catch (Exception e) { Debug.LogException(e); code = 1; }
            EditorApplication.Exit(code);
        }

        /// <summary>
        /// Batch mode starts with no scene; open the first enabled build scene
        /// (the HDRP template's OutdoorsScene: Sun, sky and exposure volume) so
        /// captures and benchmarks are lit the way the simulation will be.
        /// </summary>
        static void OpenBuildScene()
        {
            foreach (var s in EditorBuildSettings.scenes)
            {
                if (!s.enabled || !File.Exists(s.path)) continue;
                UnityEditor.SceneManagement.EditorSceneManager.OpenScene(s.path);
                Debug.Log("[Guardian] opened scene " + s.path);
                return;
            }
            Debug.LogWarning("[Guardian] no build scene found - rendering without the template's sun/sky");
        }

        /// <summary>
        /// Renders the drone's-eye view (camera spec from the manifest: stream
        /// resolution, horizontal FOV, gimbal pitch) repeatedly and reads every
        /// frame back from the GPU, as the Phase 3 bridge must. Measures the full
        /// site and the compound; writes unity_benchmark.json + one frame each.
        ///   Unity -batchmode -projectPath unity/GuardianSim -executeMethod Guardian.Sim.Editor.GuardianMenu.BatchBenchmark
        /// </summary>
        public static async void BatchBenchmark()
        {
            int code = 0;
            var results = new System.Collections.Generic.List<object>();
            string dir = null;
            try
            {
                OpenBuildScene();
                foreach (var region in new[] { SiteRegion.Compound, SiteRegion.Full })
                {
                    var loader = await Load(region);
                    dir = loader.Manifest.Directory;
                    results.Add(Benchmark(loader, warmup: 20, frames: 100));
                }
            }
            catch (Exception e) { Debug.LogException(e); code = 1; }
            if (dir != null)
            {
                var path = Path.Combine(dir, "unity_benchmark.json");
                File.WriteAllText(path, JsonConvert.SerializeObject(new
                {
                    gpu = SystemInfo.graphicsDeviceName,
                    graphics_api = SystemInfo.graphicsDeviceType.ToString(),
                    render_pipeline = GraphicsSettings.currentRenderPipeline ? GraphicsSettings.currentRenderPipeline.GetType().Name : "built-in",
                    unity = Application.unityVersion,
                    note = "each frame = render + GPU->CPU readback, like the ROS bridge will need",
                    regions = results,
                }, Formatting.Indented));
                Debug.Log("[Guardian] benchmark -> " + path);
            }
            EditorApplication.Exit(code);
        }

        static object Benchmark(SiteLoader loader, int warmup, int frames)
        {
            var cam = loader.Manifest.Camera;
            int w = cam?["stream_px"]?[0]?.ToObject<int>() ?? 960;
            int h = cam?["stream_px"]?[1]?.ToObject<int>() ?? 540;
            double hfov = cam?["hfov_deg"]?.ToObject<double>() ?? 82;
            double pitch = cam?["gimbal"]?["pitch_deg"]?.ToObject<double>() ?? -35;

            // A patrol viewpoint: 30 m above the launch pad, looking at the compound.
            var comp = loader.Manifest.Regions["compound"];
            double cx = (comp.Min[0] + comp.Max[0]) / 2, cy = (comp.Min[1] + comp.Max[1]) / 2;
            double heading = Math.Atan2(cy, cx);                       // ENU yaw towards compound centre
            var go = new GameObject("Guardian Benchmark Camera") { hideFlags = HideFlags.HideAndDontSave };
            var c = go.AddComponent<Camera>();
            c.fieldOfView = (float)(2 * Math.Atan(Math.Tan(hfov * Math.PI / 360) * h / w) * 180 / Math.PI);
            c.nearClipPlane = 0.2f; c.farClipPlane = 1500;
            go.transform.SetPositionAndRotation(Frames.EnuToUnity(0, 0, 30),
                Frames.YawZForward(heading) * Quaternion.Euler((float)-pitch, 0, 0));

            var rt = new RenderTexture(w, h, 24, RenderTextureFormat.ARGB32);
            var tex = new Texture2D(w, h, TextureFormat.RGB24, false);
            var req = new RenderPipeline.StandardRequest { destination = rt };
            void Frame()
            {
                if (RenderPipeline.SupportsRenderRequest(c, req)) RenderPipeline.SubmitRenderRequest(c, req);
                else { c.targetTexture = rt; c.Render(); c.targetTexture = null; }
                var prev = RenderTexture.active; RenderTexture.active = rt;
                tex.ReadPixels(new Rect(0, 0, w, h), 0, 0, false);   // blocks until the GPU is done
                RenderTexture.active = prev;
            }
            for (int i = 0; i < warmup; i++) Frame();
            var ms = new double[frames];
            var sw = new System.Diagnostics.Stopwatch();
            for (int i = 0; i < frames; i++) { sw.Restart(); Frame(); ms[i] = sw.Elapsed.TotalMilliseconds; }
            Array.Sort(ms);
            double mean = 0; foreach (var m in ms) mean += m; mean /= frames;

            var png = Path.Combine(loader.Manifest.Directory,
                $"unity_droneview_{loader.region.ToString().ToLowerInvariant()}.png");
            tex.Apply(); File.WriteAllBytes(png, tex.EncodeToPNG());

            long rssKb = 0;
            foreach (var line in File.ReadAllLines("/proc/self/status"))
                if (line.StartsWith("VmRSS:")) rssKb = long.Parse(line.Split((char[])null, StringSplitOptions.RemoveEmptyEntries)[1]);

            var r = new
            {
                region = loader.region.ToString(),
                site = loader.LastReport,
                resolution = new[] { w, h },
                frames,
                mean_ms = Math.Round(mean, 2),
                p50_ms = Math.Round(ms[frames / 2], 2),
                p95_ms = Math.Round(ms[(int)(frames * 0.95)], 2),
                max_ms = Math.Round(ms[frames - 1], 2),
                fps_equivalent = Math.Round(1000 / mean, 1),
                unity_rss_mb = rssKb / 1024,
                frame_png = png,
            };
            Debug.Log($"[Guardian] benchmark {loader.region}: mean {mean:F1} ms ({1000 / mean:F1} fps), " +
                      $"p95 {ms[(int)(frames * 0.95)]:F1} ms, RSS {rssKb / 1024} MB");
            UnityEngine.Object.DestroyImmediate(tex); rt.Release(); UnityEngine.Object.DestroyImmediate(rt);
            UnityEngine.Object.DestroyImmediate(go);
            return r;
        }
    }
}
