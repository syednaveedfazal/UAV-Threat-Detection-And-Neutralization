using System;
using System.IO;
using System.Net.Sockets;
using System.Text;
using System.Threading;

namespace Guardian.Sim
{
    /// <summary>Drone ground-truth pose from Gazebo (ROS/ENU).</summary>
    public struct PoseSample
    {
        public double SimTime;
        public double[] Position;   // x, y, z
        public double[] Rotation;   // qx, qy, qz, qw
        public long Seq;            // increments per received pose
    }

    /// <summary>
    /// Unity end of the camera bridge (protocol: docs/UNITY_BRIDGE.md).
    /// Connects to the ROS-side unity_bridge and retries every second, so either
    /// side can start first. Socket work runs on background threads; the main
    /// thread only reads the newest pose and hands over finished frames.
    ///
    /// Latency over completeness: only the newest pose is kept, and at most one
    /// frame waits to be sent - a slow link drops frames instead of lagging.
    /// </summary>
    public sealed class BridgeClient : IDisposable
    {
        public const uint Hello = 1, Pose = 2, Frame = 3;
        static readonly byte[] Magic = Encoding.ASCII.GetBytes("GRD1");
        public const int FrameFixedBytes = 8 + 3 * 8 + 9 * 8 + 3 * 4;   // 116, see UNITY_BRIDGE.md

        readonly string _host;
        readonly int _port;
        readonly Func<string> _helloJson;
        readonly object _lock = new();
        readonly Thread _thread;
        volatile bool _running = true;

        TcpClient _client;
        PoseSample _pose;
        bool _hasPose;
        byte[] _pendingFrame;                   // complete message, header included
        readonly AutoResetEvent _frameReady = new(false);

        public bool Connected { get; private set; }
        public string Status { get; private set; } = "starting";
        public long FramesSent, FramesDropped, PosesReceived;

        /// <param name="helloJson">called on every (re)connect, so it can describe the current camera</param>
        public BridgeClient(string host, int port, Func<string> helloJson)
        {
            _host = host; _port = port; _helloJson = helloJson;
            _thread = new Thread(Run) { IsBackground = true, Name = "Guardian bridge" };
            _thread.Start();
        }

        public bool TryGetPose(out PoseSample pose)
        {
            lock (_lock) { pose = _pose; return _hasPose; }
        }

        /// <summary>Queue a FRAME (bytes from BuildFrame). Replaces an unsent older frame.</summary>
        public void SendFrame(byte[] message)
        {
            if (!Connected) return;
            lock (_lock)
            {
                if (_pendingFrame != null) FramesDropped++;
                _pendingFrame = message;
            }
            _frameReady.Set();
        }

        /// <summary>
        /// FRAME message: header, fixed part, then RGB8 rows bottom first (the
        /// order GPU readback delivers them; the ROS side flips).
        /// </summary>
        public static byte[] BuildFrame(double simTime, GimbalPose g, int width, int height,
                                        ReadOnlySpan<byte> rgba)
        {
            int n = width * height;
            var msg = new byte[12 + FrameFixedBytes + n * 3];
            using (var w = new BinaryWriter(new MemoryStream(msg)))   // BinaryWriter is always little-endian
            {
                WriteHeader(w, Frame, (uint)(FrameFixedBytes + n * 3));
                w.Write(simTime);
                foreach (var v in g.Position) w.Write(v);
                foreach (var v in g.Right) w.Write(v);
                foreach (var v in g.Down) w.Write(v);
                foreach (var v in g.Forward) w.Write(v);
                w.Write((uint)width); w.Write((uint)height); w.Write(1u);   // encoding 1 = bottom row first
            }
            int o = 12 + FrameFixedBytes;
            for (int i = 0; i < n; i++)   // RGBA -> RGB
            {
                msg[o++] = rgba[4 * i];
                msg[o++] = rgba[4 * i + 1];
                msg[o++] = rgba[4 * i + 2];
            }
            return msg;
        }

        static void WriteHeader(BinaryWriter w, uint type, uint length)
        {
            w.Write(Magic);
            w.Write((byte)type); w.Write((byte)0); w.Write((byte)0); w.Write((byte)0);
            w.Write(length);
        }

        void Run()
        {
            while (_running)
            {
                try
                {
                    Status = $"connecting to {_host}:{_port}";
                    using var client = new TcpClient { NoDelay = true };
                    client.Connect(_host, _port);
                    lock (_lock) _client = client;
                    var stream = client.GetStream();

                    var hello = Encoding.UTF8.GetBytes(_helloJson());
                    var hw = new BinaryWriter(stream);
                    WriteHeader(hw, Hello, (uint)hello.Length); hw.Write(hello); hw.Flush();
                    Connected = true;
                    Status = $"connected to {_host}:{_port}";

                    var sender = new Thread(() => SendLoop(stream)) { IsBackground = true, Name = "Guardian bridge send" };
                    sender.Start();
                    ReadLoop(stream);           // returns when the connection ends
                }
                catch (Exception e) when (e is SocketException || e is IOException || e is ObjectDisposedException)
                {
                    if (Connected) Status = "connection lost: " + e.Message;
                    else Status = $"waiting for unity_bridge on {_host}:{_port} ({e.Message})";
                }
                finally
                {
                    Connected = false;
                    lock (_lock) { _client = null; _pendingFrame = null; }
                    _frameReady.Set();          // wake the sender so it exits
                }
                if (_running) Thread.Sleep(1000);
            }
        }

        void ReadLoop(NetworkStream stream)
        {
            var header = new byte[12];
            while (_running)
            {
                ReadExactly(stream, header, 12);
                if (header[0] != Magic[0] || header[1] != Magic[1] || header[2] != Magic[2] || header[3] != Magic[3])
                    throw new IOException("stream out of sync (bad magic)");
                uint type = header[4];
                int len = (int)BitConverter.ToUInt32(header, 8);
                if (len < 0 || len > 64 << 20) throw new IOException($"payload too large: {len}");
                var payload = new byte[len];
                ReadExactly(stream, payload, len);
                if (type == Pose && len == 64)
                {
                    var p = new PoseSample
                    {
                        SimTime = BitConverter.ToDouble(payload, 0),
                        Position = new[] { BitConverter.ToDouble(payload, 8), BitConverter.ToDouble(payload, 16), BitConverter.ToDouble(payload, 24) },
                        Rotation = new[] { BitConverter.ToDouble(payload, 32), BitConverter.ToDouble(payload, 40),
                                           BitConverter.ToDouble(payload, 48), BitConverter.ToDouble(payload, 56) },
                    };
                    lock (_lock) { p.Seq = _pose.Seq + 1; _pose = p; _hasPose = true; }
                    PosesReceived++;
                }
            }
        }

        void SendLoop(NetworkStream stream)
        {
            while (_running && Connected)
            {
                _frameReady.WaitOne(500);
                byte[] msg;
                lock (_lock) { msg = _pendingFrame; _pendingFrame = null; }
                if (msg == null) continue;
                try { stream.Write(msg, 0, msg.Length); FramesSent++; }
                catch (Exception) { CloseSocket(); return; }   // the read loop notices and reconnects
            }
        }

        static void ReadExactly(NetworkStream s, byte[] buf, int n)
        {
            int got = 0;
            while (got < n)
            {
                int r = s.Read(buf, got, n - got);
                if (r <= 0) throw new IOException("unity_bridge closed the connection");
                got += r;
            }
        }

        void CloseSocket()
        {
            lock (_lock) _client?.Close();
        }

        public void Dispose()
        {
            _running = false;
            CloseSocket();
            _frameReady.Set();
        }
    }
}
