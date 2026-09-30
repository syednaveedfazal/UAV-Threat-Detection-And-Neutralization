using System.Collections.Generic;
using UnityEngine;

namespace Guardian.Sim
{
    /// <summary>
    /// Simple stand-in shapes for manifest instances (Phase 2). Phase 4 swaps these
    /// for realistic prefabs; positions and sizes stay exactly the same.
    /// </summary>
    public static class Placeholders
    {
        static readonly Dictionary<Color, Material> Cache = new();

        public static Material Mat(Color c)
        {
            if (Cache.TryGetValue(c, out var m) && m) return m;
            var shader = Shader.Find("HDRP/Lit") ?? Shader.Find("Universal Render Pipeline/Lit")
                         ?? Shader.Find("Standard");
            m = new Material(shader) { color = c };
            if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", c);
            Cache[c] = m;
            return m;
        }

        static GameObject Prim(PrimitiveType type, Transform parent, string name, Color c, bool collider)
        {
            var go = GameObject.CreatePrimitive(type);
            go.name = name;
            go.transform.SetParent(parent, false);
            go.GetComponent<Renderer>().sharedMaterial = Mat(c);
            if (!collider) Destroy(go.GetComponent<Collider>());
            return go;
        }

        /// <summary>Box from a manifest perimeter part: size = [length along yaw, width, height].</summary>
        public static GameObject Box(Transform parent, Part p, Color c)
        {
            var go = Prim(PrimitiveType.Cube, parent, p.Type, c, collider: true);
            go.transform.SetPositionAndRotation(
                Frames.EnuToUnity(p.Centre[0], p.Centre[1], p.Centre[2]), Frames.YawXAligned(p.Yaw));
            go.transform.localScale = new Vector3((float)p.Size[0], (float)p.Size[2], (float)p.Size[1]);
            return go;
        }

        /// <summary>Tree from [x, y, z_ground, height, crown radius]: trunk + ellipsoid crown.</summary>
        public static GameObject Tree(Transform parent, double[] t, int index)
        {
            float h = (float)t[3], r = (float)t[4];
            var root = new GameObject($"tree_{index}");
            root.transform.SetParent(parent, false);
            root.transform.position = Frames.EnuToUnity(t[0], t[1], t[2]);
            float trunkH = 0.4f * h, trunkR = Mathf.Max(0.15f, 0.025f * h);
            var trunk = Prim(PrimitiveType.Cylinder, root.transform, "trunk", new Color(0.35f, 0.25f, 0.15f), false);
            trunk.transform.localPosition = new Vector3(0, trunkH / 2, 0);
            trunk.transform.localScale = new Vector3(2 * trunkR, trunkH / 2, 2 * trunkR);   // cylinder is 2 units tall
            var crown = Prim(PrimitiveType.Sphere, root.transform, "crown", new Color(0.22f, 0.42f, 0.18f), false);
            float crownH = h - trunkH + 0.6f;
            crown.transform.localPosition = new Vector3(0, trunkH + (h - trunkH) / 2, 0);
            crown.transform.localScale = new Vector3(2 * r, crownH, 2 * r);
            return root;
        }

        /// <summary>A person-sized capsule with its pivot at the feet, facing local +Z.</summary>
        public static GameObject Person(Transform parent, string name, Color c)
        {
            var root = new GameObject(name);
            root.transform.SetParent(parent, false);
            var body = Prim(PrimitiveType.Capsule, root.transform, "body", c, false);
            body.transform.localPosition = new Vector3(0, 0.875f, 0);
            body.transform.localScale = new Vector3(0.5f, 0.875f, 0.35f);   // 1.75 m tall
            var nose = Prim(PrimitiveType.Cube, root.transform, "facing", Color.black, false);
            nose.transform.localPosition = new Vector3(0, 1.55f, 0.2f);      // shows walking direction
            nose.transform.localScale = new Vector3(0.12f, 0.08f, 0.12f);
            return root;
        }

        internal static void Destroy(Object o)
        {
            if (!o) return;
            if (Application.isPlaying) Object.Destroy(o); else Object.DestroyImmediate(o);
        }
    }
}
