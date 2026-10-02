using System;
using UnityEngine;

namespace Guardian.Sim
{
    /// <summary>
    /// Player builds: set up the site and the drone camera from the command line,
    /// so the HDRP template scene (sun, sky, exposure) is used unmodified.
    ///
    ///   GuardianSim.x86_64 [--scene manifest.json] [--region compound|full]
    ///                      [--bridge-host 127.0.0.1] [--bridge-port 5700] [--no-feed]
    ///
    /// In the editor use Guardian > Play Drone Camera instead.
    /// </summary>
    public static class GuardianBootstrap
    {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        static void OnLoad()
        {
            if (Application.isEditor) return;
            var args = Environment.GetCommandLineArgs();
            int i = Array.IndexOf(args, "--region");
            var region = i >= 0 && i + 1 < args.Length && args[i + 1] == "full" ? SiteRegion.Full : SiteRegion.Compound;

            var site = UnityEngine.Object.FindAnyObjectByType<SiteLoader>();
            if (site == null)
            {
                site = new GameObject("Guardian Site").AddComponent<SiteLoader>();
                site.region = region;
            }
            if (UnityEngine.Object.FindAnyObjectByType<DroneRig>() == null)
            {
                var rig = new GameObject("Guardian Drone").AddComponent<DroneRig>();
                rig.site = site;
                rig.showFeed = Array.IndexOf(args, "--no-feed") < 0;
            }
            if (Camera.main != null) Camera.main.farClipPlane = 2000;
            Debug.Log($"[Guardian] player bootstrap: region {region}");
        }
    }
}
