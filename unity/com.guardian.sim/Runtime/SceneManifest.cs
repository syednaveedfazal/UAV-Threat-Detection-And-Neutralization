using System.Collections.Generic;
using System.IO;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace Guardian.Sim
{
    /// <summary>
    /// Typed view of scene_manifest.json (schema: scripts/scene_manifest.schema.json).
    /// All positions are ENU metres, origin at the launch pad. Convert with Frames.
    /// </summary>
    public class SceneManifest
    {
        [JsonProperty("manifest_version")] public int Version;
        [JsonProperty("site")] public string Site;
        [JsonProperty("frame")] public FrameInfo Frame;
        [JsonProperty("regions")] public Dictionary<string, Box2> Regions;
        [JsonProperty("assets")] public Assets Assets;
        [JsonProperty("buildings")] public List<Building> Buildings;
        /// <summary>[x, y, z_ground, height_m, crown_radius_m]</summary>
        [JsonProperty("trees")] public List<double[]> Trees;
        [JsonProperty("perimeter")] public List<Part> Perimeter;
        [JsonProperty("actors")] public List<Actor> Actors;
        [JsonProperty("launch_pad")] public double[] LaunchPad;
        [JsonProperty("camera")] public JObject Camera;
        [JsonProperty("sun")] public SunInfo Sun;
        [JsonProperty("stats")] public JObject Stats;

        /// <summary>Directory holding the manifest and the .glb files.</summary>
        [JsonIgnore] public string Directory;

        /// <summary>
        /// Keep date-like strings (sun local_time) as plain text. Newtonsoft's default
        /// silently converts them to DateTime in the machine's time zone.
        /// </summary>
        public static readonly JsonSerializerSettings Settings = new() { DateParseHandling = DateParseHandling.None };

        public static SceneManifest Load(string path)
        {
            var m = JsonConvert.DeserializeObject<SceneManifest>(File.ReadAllText(path), Settings);
            if (m == null || m.Version != 1)
                throw new InvalidDataException($"{path}: expected manifest_version 1");
            m.Directory = System.IO.Path.GetDirectoryName(System.IO.Path.GetFullPath(path));
            return m;
        }
    }

    public class FrameInfo
    {
        [JsonProperty("origin_wgs84")] public Wgs84 OriginWgs84;
        [JsonProperty("bbox_world")] public JObject BboxWorld;
    }

    public class Wgs84
    {
        [JsonProperty("lat")] public double Lat;
        [JsonProperty("lon")] public double Lon;
        [JsonProperty("elevation_m")] public double ElevationM;
    }

    public class Box2
    {
        [JsonProperty("min")] public double[] Min;
        [JsonProperty("max")] public double[] Max;

        public bool Contains(double x, double y, double margin = 0) =>
            x >= Min[0] - margin && x <= Max[0] + margin &&
            y >= Min[1] - margin && y <= Max[1] + margin;

        public bool Overlaps(double minX, double minY, double size) =>
            minX <= Max[0] && minX + size >= Min[0] && minY <= Max[1] && minY + size >= Min[1];
    }

    public class Assets
    {
        [JsonProperty("terrain")] public TerrainAsset Terrain;
        [JsonProperty("buildings")] public BuildingsAsset Buildings;
        [JsonProperty("ndvi")] public JObject Ndvi;
    }

    public class TerrainAsset
    {
        [JsonProperty("file")] public string File;
        [JsonProperty("tiles")] public List<Tile> Tiles;
    }

    public class Tile
    {
        [JsonProperty("name")] public string Name;
        [JsonProperty("min")] public double[] Min;
        [JsonProperty("size_m")] public double SizeM;
        [JsonProperty("texture_px")] public int TexturePx;
    }

    public class BuildingsAsset
    {
        [JsonProperty("file")] public string File;
        [JsonProperty("nodes")] public List<string> Nodes;
    }

    public class Building
    {
        [JsonProperty("id")] public string Id;
        [JsonProperty("category")] public string Category;
        [JsonProperty("function")] public string Function;
        [JsonProperty("roof_type")] public string RoofType;
        [JsonProperty("height_m")] public double? HeightM;
        [JsonProperty("centroid")] public double[] Centroid;
        [JsonProperty("source")] public string Source;
    }

    /// <summary>A box part of the perimeter: size = [length along yaw, width, height].</summary>
    public class Part
    {
        [JsonProperty("type")] public string Type;
        [JsonProperty("centre")] public double[] Centre;
        [JsonProperty("size")] public double[] Size;
        [JsonProperty("yaw")] public double Yaw;
    }

    public class Actor
    {
        [JsonProperty("name")] public string Name;
        [JsonProperty("speed")] public double Speed;
        [JsonProperty("loop")] public bool Loop;
        [JsonProperty("waypoints")] public List<Waypoint> Waypoints;
    }

    public class Waypoint
    {
        [JsonProperty("t")] public double T;
        [JsonProperty("x")] public double X;
        [JsonProperty("y")] public double Y;
        [JsonProperty("z_ground")] public double ZGround;
        [JsonProperty("yaw")] public double Yaw;
    }

    public class SunInfo
    {
        [JsonProperty("local_time")] public string LocalTime;
        [JsonProperty("timezone")] public string Timezone;
    }
}
