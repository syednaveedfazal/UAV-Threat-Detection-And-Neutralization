using System.Collections;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using NUnit.Framework;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.TestTools;

namespace Guardian.Sim.Tests
{
    /// <summary>
    /// Loads the real Bonn scene package and checks it lands in the right place.
    /// Skipped (Inconclusive) when no package has been built.
    /// </summary>
    public class SiteTests
    {
        static string ManifestPath => SiteLoader.ResolveManifestPath("");

        static SceneManifest RequireManifest()
        {
            if (!File.Exists(ManifestPath)) Assert.Inconclusive($"no scene package at {ManifestPath}");
            return SceneManifest.Load(ManifestPath);
        }

        [Test]
        public void ManifestCountsMatchStats()
        {
            var m = RequireManifest();
            Assert.AreEqual((int)m.Stats["buildings"], m.Buildings.Count);
            Assert.AreEqual((int)m.Stats["trees"], m.Trees.Count);
            Assert.AreEqual((int)m.Stats["terrain_tiles"], m.Assets.Terrain.Tiles.Count);
            Assert.That(m.Actors.Select(a => a.Name), Does.Contain("intruder"));
        }

        [Test]
        public void ActorPathHitsEveryWaypointAndLoops()
        {
            var m = RequireManifest();
            foreach (var a in m.Actors)
            {
                foreach (var w in a.Waypoints)
                {
                    var (x, y, z, _) = ActorPath.Evaluate(a.Waypoints, w.T, a.Loop);
                    Assert.AreEqual(w.X, x, 1e-6); Assert.AreEqual(w.Y, y, 1e-6); Assert.AreEqual(w.ZGround, z, 1e-6);
                }
                double period = a.Waypoints[^1].T;
                var p0 = ActorPath.Evaluate(a.Waypoints, 1.0, a.Loop);
                var p1 = ActorPath.Evaluate(a.Waypoints, 1.0 + period, a.Loop);
                Assert.AreEqual(p0.x, p1.x, 1e-6, "looped path repeats after one period");
            }
        }

        [Test]
        public void ActorPathInterpolatesBetweenWaypoints()
        {
            var w = new[] {
                new Waypoint { T = 0, X = 0, Y = 0, ZGround = 0, Yaw = 0 },
                new Waypoint { T = 10, X = 10, Y = 0, ZGround = 1, Yaw = 0 } };
            var (x, _, z, _) = ActorPath.Evaluate(w, 2.5, false);
            Assert.AreEqual(2.5, x, 1e-9); Assert.AreEqual(0.25, z, 1e-9);
        }

        [UnityTest]
        public IEnumerator CompoundLoadsInTheRightPlace()
        {
            var m = RequireManifest();
            EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);
            var loader = new GameObject("Guardian Site").AddComponent<SiteLoader>();
            loader.region = SiteRegion.Compound;
            var task = loader.LoadAsync();                // also runs VerifyOrientation
            while (!task.IsCompleted) yield return null;
            if (task.IsFaulted) throw task.Exception!.GetBaseException();

            // 1. The launch pad (world origin) is on the ground: terrain at ~0 m.
            Assert.IsTrue(Physics.Raycast(new Vector3(0, 100, 0), Vector3.down, out var hit, 200), "terrain under the launch pad");
            Assert.AreEqual(0f, hit.point.y, 0.5f, "launch pad height");

            // 2. The new institute building sits where the manifest says, roof ~17 m up.
            //    Mirrored or rotated geometry would put bare ground here instead.
            var b = m.Buildings.First(x => x.Id == "new_institute_building");
            var above = Frames.EnuToUnity(b.Centroid[0], b.Centroid[1], 200);
            Assert.IsTrue(Physics.Raycast(above, Vector3.down, out hit, 400), "something under the building centroid");
            Assert.AreEqual((float)b.HeightM.Value, hit.point.y, 2.0f, "institute roof height");

            // 3. Perimeter and people exist.
            Assert.Greater(loader.SiteRoot.GetComponentsInChildren<Transform>().Count(t => t.name == "fence"), 50);
            Assert.IsNotNull(SiteLoader.FindDeep(loader.SiteRoot, "intruder"));
            loader.Unload();
        }
    }
}
