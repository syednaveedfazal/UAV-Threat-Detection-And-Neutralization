using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using GLTFast;
using UnityEngine;

namespace Guardian.Sim
{
    public enum SiteRegion { Compound, Full }

    /// <summary>
    /// Builds the real site from the engine-neutral scene package
    /// (scripts/build_real_site.py --export-scene). The same code runs in the
    /// editor (Guardian menu) and in a player build, so what you inspect is what
    /// the simulation renders.
    ///
    /// Manifest path, first match wins: command line "--scene PATH", environment
    /// variable GUARDIAN_SCENE, the manifestPath field, then DefaultManifest.
    /// </summary>
    public class SiteLoader : MonoBehaviour
    {
        public const string DefaultManifest = "~/UAV/data/bonn_poppelsdorf/world/scene/scene_manifest.json";

        [Tooltip("scene_manifest.json; empty = --scene / GUARDIAN_SCENE / default")]
        public string manifestPath = "";
        public SiteRegion region = SiteRegion.Compound;
        public bool loadOnStart = true;
        public bool applySun = true;

        public SceneManifest Manifest { get; private set; }
        public Transform SiteRoot { get; private set; }
        public string LastReport { get; private set; } = "";
        /// <summary>True once LoadAsync has finished building the site (false again after Unload).</summary>
        public bool IsLoaded { get; private set; }

        readonly List<GltfImport> _imports = new();

        async void Start()
        {
            if (!loadOnStart || !Application.isPlaying) return;
            try { await LoadAsync(); }
            catch (Exception e) { Debug.LogException(e); }
        }

        void OnDestroy() => DisposeImports();

        public static string ResolveManifestPath(string configured)
        {
            var args = Environment.GetCommandLineArgs();
            int i = Array.IndexOf(args, "--scene");
            string p = i >= 0 && i + 1 < args.Length ? args[i + 1]
                     : !string.IsNullOrEmpty(Environment.GetEnvironmentVariable("GUARDIAN_SCENE"))
                       ? Environment.GetEnvironmentVariable("GUARDIAN_SCENE")
                     : !string.IsNullOrEmpty(configured) ? configured : DefaultManifest;
            if (p.StartsWith("~"))
                p = Environment.GetEnvironmentVariable("HOME") + p.Substring(1);
            return Path.GetFullPath(p);
        }

        public Box2 RegionBox => Manifest.Regions[region == SiteRegion.Compound ? "compound" : "full"];

        public async Task LoadAsync()
        {
            var path = ResolveManifestPath(manifestPath);
            if (!File.Exists(path))
                throw new FileNotFoundException(
                    $"Scene manifest not found: {path}\nBuild it with: scripts/build_real_site.py <site.yaml> --export-scene", path);
            Manifest = SceneManifest.Load(path);
            var box = RegionBox;

            Unload();
            SiteRoot = new GameObject($"Site {Manifest.Site} ({region})").transform;
            SiteRoot.SetParent(transform, false);

            // glTF meshes: see Frames.GltfRootCorrection for why this node is turned 180 deg.
            var gltfRoot = new GameObject("glTF").transform;
            gltfRoot.SetParent(SiteRoot, false);
            gltfRoot.localRotation = Frames.GltfRootCorrection;
            await LoadGlb(Path.Combine(Manifest.Directory, Manifest.Assets.Terrain.File), gltfRoot);
            await LoadGlb(Path.Combine(Manifest.Directory, Manifest.Assets.Buildings.File), gltfRoot);

            int tilesOn = 0;
            foreach (var tile in Manifest.Assets.Terrain.Tiles)
            {
                var t = FindDeep(gltfRoot, tile.Name);
                if (t == null) throw new InvalidDataException($"terrain.glb has no node '{tile.Name}'");
                bool on = box.Overlaps(tile.Min[0], tile.Min[1], tile.SizeM);
                t.gameObject.SetActive(on);
                if (on) { AddMeshColliders(t); tilesOn++; }
            }
            foreach (var node in Manifest.Assets.Buildings.Nodes)
            {
                var b = FindDeep(gltfRoot, node);
                if (b) AddMeshColliders(b);
            }
            VerifyOrientation(gltfRoot);

            var inst = new GameObject("Instances").transform;
            inst.SetParent(SiteRoot, false);
            var trees = new GameObject("Trees").transform; trees.SetParent(inst, false);
            int nTrees = 0;
            for (int k = 0; k < Manifest.Trees.Count; k++)
            {
                var t = Manifest.Trees[k];
                if (!box.Contains(t[0], t[1])) continue;
                Placeholders.Tree(trees, t, k); nTrees++;
            }

            var perim = new GameObject("Perimeter").transform; perim.SetParent(inst, false);
            int nParts = 0;
            foreach (var p in Manifest.Perimeter)
            {
                if (!box.Contains(p.Centre[0], p.Centre[1])) continue;
                Placeholders.Box(perim, p, PartColor(p.Type)); nParts++;
            }

            var people = new GameObject("People").transform; people.SetParent(inst, false);
            foreach (var a in Manifest.Actors)
            {
                var color = a.Name.Contains("intruder") ? new Color(0.8f, 0.15f, 0.1f) : new Color(0.2f, 0.35f, 0.8f);
                var person = Placeholders.Person(people, a.Name, color);
                person.AddComponent<ActorPath>().Init(a);
            }

            string sun = applySun ? ApplySun() : "sun: not applied";
            SetDontSave(SiteRoot.gameObject);   // generated content never gets saved into a scene
            Physics.SyncTransforms();
            LastReport = $"{Manifest.Site} [{region}]: {tilesOn}/{Manifest.Assets.Terrain.Tiles.Count} terrain tiles, " +
                         $"{Manifest.Assets.Buildings.Nodes.Count} building meshes, {nTrees} trees, " +
                         $"{nParts} perimeter parts, {Manifest.Actors.Count} people; {sun}";
            Debug.Log("[Guardian] " + LastReport);
            IsLoaded = true;
        }

        public void Unload()
        {
            IsLoaded = false;
            if (SiteRoot) Placeholders.Destroy(SiteRoot.gameObject);
            SiteRoot = null;
            DisposeImports();
        }

        async Task LoadGlb(string file, Transform parent)
        {
            // UninterruptedDeferAgent: load in one go (also works in edit mode / tests).
            var gltf = new GltfImport(deferAgent: new UninterruptedDeferAgent());
            _imports.Add(gltf);
            if (!await gltf.Load(new Uri(Path.GetFullPath(file))))
                throw new IOException($"glTFast could not load {file}");
            if (!await gltf.InstantiateMainSceneAsync(parent))
                throw new IOException($"glTFast could not instantiate {file}");
        }

        /// <summary>
        /// The south-west terrain tile must land where the manifest says, i.e. at
        /// -X (west) / -Z (south) of the origin for this site. A wrong axis
        /// convention would put it mirrored or rotated - fail loudly instead.
        /// </summary>
        void VerifyOrientation(Transform gltfRoot)
        {
            var tile = Manifest.Assets.Terrain.Tiles.First();
            var t = FindDeep(gltfRoot, tile.Name);
            var r = t ? t.GetComponentInChildren<Renderer>(true) : null;
            if (r == null) throw new InvalidDataException("cannot verify orientation: no terrain renderer");
            var expected = Frames.EnuToUnity(tile.Min[0] + tile.SizeM / 2, tile.Min[1] + tile.SizeM / 2, 0);
            var got = r.bounds.center;
            float err = new Vector2(got.x - expected.x, got.z - expected.z).magnitude;
            if (err > 2f)
                throw new InvalidDataException(
                    $"glTF orientation mismatch: {tile.Name} centre is at Unity ({got.x:F1}, {got.z:F1}), " +
                    $"expected ({expected.x:F1}, {expected.z:F1}). Check Frames.GltfRootCorrection.");
        }

        string ApplySun()
        {
            if (Manifest.Sun == null || string.IsNullOrEmpty(Manifest.Sun.LocalTime)) return "sun: none in manifest";
            var utc = SolarPosition.LocalToUtc(Manifest.Sun.LocalTime, Manifest.Sun.Timezone ?? "UTC");
            var o = Manifest.Frame.OriginWgs84;
            var (el, az) = SolarPosition.Compute(utc, o.Lat, o.Lon);
            var light = FindObjectsByType<Light>()
                        .FirstOrDefault(l => l.type == LightType.Directional);
            if (light == null) return $"sun elevation {el:F1} deg, azimuth {az:F1} deg (no directional light in scene)";
            light.transform.rotation = SolarPosition.LightRotation(el, az);
            return $"sun elevation {el:F1} deg, azimuth {az:F1} deg ({Manifest.Sun.LocalTime} {Manifest.Sun.Timezone})";
        }

        static Color PartColor(string type) => type switch
        {
            "fence" => new Color(0.18f, 0.30f, 0.22f),
            "post" => new Color(0.2f, 0.2f, 0.2f),
            "gate" => new Color(0.75f, 0.62f, 0.10f),
            "booth" => new Color(0.85f, 0.85f, 0.82f),
            "booth_roof" => new Color(0.25f, 0.25f, 0.28f),
            "window" => new Color(0.25f, 0.35f, 0.45f),
            "pole" => new Color(0.5f, 0.5f, 0.5f),
            "lamp" => new Color(1.0f, 0.95f, 0.7f),
            _ => Color.magenta,
        };

        static void AddMeshColliders(Transform t)
        {
            foreach (var mf in t.GetComponentsInChildren<MeshFilter>(true))
                if (!mf.GetComponent<MeshCollider>())
                    mf.gameObject.AddComponent<MeshCollider>().sharedMesh = mf.sharedMesh;
        }

        public static Transform FindDeep(Transform root, string name)
        {
            if (root.name == name) return root;
            foreach (Transform c in root)
            {
                var r = FindDeep(c, name);
                if (r) return r;
            }
            return null;
        }

        static void SetDontSave(GameObject go)
        {
            go.hideFlags |= HideFlags.DontSave;
            foreach (Transform c in go.transform) SetDontSave(c.gameObject);
        }

        void DisposeImports()
        {
            foreach (var g in _imports) g?.Dispose();
            _imports.Clear();
        }
    }
}
