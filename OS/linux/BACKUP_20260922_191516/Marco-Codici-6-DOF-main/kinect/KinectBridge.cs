using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using Microsoft.Kinect;

namespace Robot6DOF
{
    // Kinect for Windows SDK 1.8 -> stream RGB + profondita su HTTP locale (porta 8766)
    // Endpoint: /status  /color.jpg  /depth.bin  /map?x=&y=
    static class KinectBridge
    {
        const int W = 640;
        const int H = 480;
        const int N = W * H;
        const int Port = 8766;

        static KinectSensor g_sensor;
        static byte[] g_jpeg;
        static readonly object g_colorLock = new object();

        static ushort[] g_depth = new ushort[N];
        static ushort[] g_scratch = new ushort[N];
        static readonly object g_depthLock = new object();

        static long g_colorFrames;
        static long g_depthFrames;
        static double g_colorFps;
        static double g_depthFps;
        static int g_colorFpsCount;
        static int g_depthFpsCount;
        static DateTime g_colorFpsWin = DateTime.UtcNow;
        static DateTime g_depthFpsWin = DateTime.UtcNow;
        static DateTime g_lastJpeg = DateTime.MinValue;

        static int g_statMin;
        static int g_statMax;
        static int g_statValid;
        static double g_statAvg;
        static int g_centerDepth;
        static float g_centerX;
        static float g_centerY;
        static float g_centerZ;

        static float g_dFx;
        static float g_dFy;
        static float g_cFx;
        static float g_cFy;
        static int g_minDepth;
        static int g_maxDepth;
        static int g_tooNear;
        static int g_tooFar;
        static int g_unknown;
        static string g_deviceId = "";

        static volatile bool g_running;
        static volatile string g_error = "";

        static long g_lastFrameTicksUtc;
        static long g_nullFrames;
        static int g_restarts;

        static int g_elevTarget;
        static string g_elevMsg = "";
        static int g_elevPending = int.MinValue;

        const int AudioRate = 16000;
        const int AudioBytesPerMs = 32;
        static Stream g_audioStream;
        static volatile bool g_audioRunning;
        static string g_audioError = "";
        static byte[] g_audioRing = new byte[AudioRate * 2 * 15];
        static int g_audioRingLen;
        static int g_audioRingPos;
        static long g_audioBytes;
        static int g_audioRmsM;
        static readonly object g_audioLock = new object();

        static void StartAudio(KinectSensor s)
        {
            try
            {
                KinectAudioSource src = s.AudioSource;
                src.BeamAngleMode = BeamAngleMode.Adaptive;
                src.NoiseSuppression = true;
                src.AutomaticGainControlEnabled = true;
                g_audioStream = src.Start();
                g_audioRunning = true;
                g_audioError = "";
                g_audioBytes = 0;
                g_audioRmsM = 0;
                lock (g_audioLock) { g_audioRingLen = 0; g_audioRingPos = 0; }
                Thread at = new Thread(AudioLoop);
                at.IsBackground = true;
                at.Start();
                Console.WriteLine("Mic array avviato: 16 kHz mono PCM, beam adaptive");
            }
            catch (Exception ex)
            {
                g_audioRunning = false;
                g_audioError = ex.Message;
                g_audioStream = null;
                Console.WriteLine("Errore audio: " + ex.Message);
            }
        }

        static void StopAudio()
        {
            g_audioRunning = false;
            try { if (g_audioStream != null) g_audioStream.Close(); }
            catch { }
            g_audioStream = null;
            try { if (g_sensor != null) g_sensor.AudioSource.Stop(); }
            catch { }
            lock (g_audioLock) { g_audioRingLen = 0; g_audioRingPos = 0; }
        }

        static void AudioLoop()
        {
            byte[] buf = new byte[8192];
            Stream st = g_audioStream;
            while (g_audioRunning && st != null)
            {
                int n;
                try { n = st.Read(buf, 0, buf.Length); }
                catch { break; }
                if (n <= 0) { Thread.Sleep(20); continue; }
                AppendRing(buf, n);
                double sum = 0;
                int cnt = n / 2;
                for (int i = 0; i + 1 < n; i += 2)
                {
                    short sv = (short)(buf[i] | (buf[i + 1] << 8));
                    sum += (double)sv * sv;
                }
                double rms = cnt > 0 ? Math.Sqrt(sum / cnt) / 32768.0 : 0;
                int smooth = (int)(g_audioRmsM * 0.85 + rms * 1500.0);
                Interlocked.Exchange(ref g_audioRmsM, smooth);
            }
            g_audioRunning = false;
        }

        static void AppendRing(byte[] buf, int n)
        {
            lock (g_audioLock)
            {
                for (int i = 0; i < n; i++)
                {
                    g_audioRing[g_audioRingPos] = buf[i];
                    g_audioRingPos = (g_audioRingPos + 1) % g_audioRing.Length;
                }
                g_audioRingLen = Math.Min(g_audioRingLen + n, g_audioRing.Length);
                g_audioBytes += n;
            }
        }

        static byte[] ReadRing(int byteCount)
        {
            if (byteCount > g_audioRing.Length) byteCount = g_audioRing.Length;
            byteCount -= byteCount % 2;
            byte[] outb = new byte[byteCount];
            lock (g_audioLock)
            {
                if (byteCount > g_audioRingLen) byteCount = g_audioRingLen - (g_audioRingLen % 2);
                if (byteCount <= 0) return new byte[0];
                outb = new byte[byteCount];
                int start = (g_audioRingPos - byteCount + g_audioRing.Length * 2) % g_audioRing.Length;
                if (start + byteCount <= g_audioRing.Length)
                {
                    Buffer.BlockCopy(g_audioRing, start, outb, 0, byteCount);
                }
                else
                {
                    int first = g_audioRing.Length - start;
                    Buffer.BlockCopy(g_audioRing, start, outb, 0, first);
                    Buffer.BlockCopy(g_audioRing, 0, outb, first, byteCount - first);
                }
            }
            return outb;
        }

        static byte[] WrapWav(byte[] pcm)
        {
            byte[] h = new byte[44];
            int dataLen = pcm.Length;
            Encoding.ASCII.GetBytes("RIFF").CopyTo(h, 0);
            BitConverter.GetBytes(36 + dataLen).CopyTo(h, 4);
            Encoding.ASCII.GetBytes("WAVE").CopyTo(h, 8);
            Encoding.ASCII.GetBytes("fmt ").CopyTo(h, 12);
            BitConverter.GetBytes(16).CopyTo(h, 16);
            BitConverter.GetBytes((short)1).CopyTo(h, 20);
            BitConverter.GetBytes((short)1).CopyTo(h, 22);
            BitConverter.GetBytes(AudioRate).CopyTo(h, 24);
            BitConverter.GetBytes(AudioRate * 2).CopyTo(h, 28);
            BitConverter.GetBytes((short)2).CopyTo(h, 32);
            BitConverter.GetBytes((short)16).CopyTo(h, 34);
            Encoding.ASCII.GetBytes("data").CopyTo(h, 36);
            BitConverter.GetBytes(dataLen).CopyTo(h, 40);
            byte[] all = new byte[44 + dataLen];
            Buffer.BlockCopy(h, 0, all, 0, 44);
            Buffer.BlockCopy(pcm, 0, all, 44, dataLen);
            return all;
        }

        static string BuildAudioFields()
        {
            double beam = 0, srcA = 0, conf = 0;
            string err = g_audioError;
            try
            {
                if (g_sensor != null && g_sensor.IsRunning)
                {
                    KinectAudioSource a = g_sensor.AudioSource;
                    beam = a.BeamAngle;
                    srcA = a.SoundSourceAngle;
                    conf = a.SoundSourceAngleConfidence;
                }
            }
            catch (Exception ex) { if (err == "") err = ex.Message; }
            int bufferedMs;
            lock (g_audioLock) { bufferedMs = (int)(g_audioRingLen * 1000L / (AudioRate * 2)); }
            StringBuilder sb = new StringBuilder(256);
            sb.Append("\"running\":").Append(g_audioRunning ? "true" : "false");
            sb.Append(",\"beam\":").Append(D1(beam));
            sb.Append(",\"source\":").Append(D1(srcA));
            sb.Append(",\"conf\":").Append(D1(conf));
            sb.Append(",\"rms\":").Append((g_audioRmsM / 10000.0).ToString("0.0000", System.Globalization.CultureInfo.InvariantCulture));
            sb.Append(",\"bytes\":").Append(g_audioBytes);
            sb.Append(",\"bufferedMs\":").Append(bufferedMs);
            sb.Append(",\"sampleRate\":").Append(AudioRate);
            sb.Append(",\"channels\":1");
            sb.Append(",\"error\":\"").Append(Escape(err)).Append("\"");
            return sb.ToString();
        }

        /* ================= SCAN / POSE TRACKING ================= */

        const int StateOff = 0;
        const int StateOk = 1;
        const int StateWeak = 2;
        const int StateLost = 3;
        const int PoseHeaderBytes = 96;
        const int MaxKf = 600;
        const float ModelCellSize = 80f;
        const int MaxModelCells = 400000;
        const float IcpCorr = 350f;

        static volatile bool g_trackOn;
        static readonly object g_trackLock = new object();
        static AutoResetEvent g_trackPulse = new AutoResetEvent(false);
        static double[] g_pose = IdentT();
        static double[] g_lastPose = IdentT();
        static double[] g_prevPose = IdentT();
        static int g_trackState;
        static int g_trackInliers;
        static float g_trackRmse;
        static long g_trackSeq;
        static int g_icpMainOk;
        static int g_icpMainFail;
        static bool g_kfBootstrapped;
        static List<ScanKf> g_kfs = new List<ScanKf>();
        static Dictionary<long, int> g_modelCells = new Dictionary<long, int>();
        static List<ModelCell> g_modelList = new List<ModelCell>();
        static List<LoopEdge> g_loops = new List<LoopEdge>();
        static int g_optSeq;
        static double[] g_deltas = new double[0];
        static int g_deltaOptSeq = -1;

        sealed class ScanKf
        {
            public float[] pts;
            public float[] nrm;
            public int n;
            public double[] pose = IdentT();
            public double[] meas = IdentT();
            public float[] desc = new float[128];
        }

        sealed class ModelCell
        {
            public float x, y, z;
            public float nx, ny, nz;
            public int cnt;
        }

        sealed class LoopEdge
        {
            public int i;
            public int j;
            public double[] z = IdentT();
        }

        static double[] IdentT()
        {
            return new double[16] {
                1,0,0,0,
                0,1,0,0,
                0,0,1,0,
                0,0,0,1
            };
        }

        static void SanitizePose(double[] p)
        {
            double r00 = p[0], r01 = p[1], r02 = p[2];
            double r10 = p[4], r11 = p[5], r12 = p[6];
            double r20 = p[8], r21 = p[9], r22 = p[10];
            double n0 = Math.Sqrt(r00 * r00 + r01 * r01 + r02 * r02);
            if (n0 < 1e-9 || double.IsNaN(n0))
            {
                double[] id = IdentT();
                Array.Copy(id, p, 16);
                return;
            }
            r00 /= n0; r01 /= n0; r02 /= n0;
            double d = r10 * r00 + r11 * r01 + r12 * r02;
            r10 -= d * r00; r11 -= d * r01; r12 -= d * r02;
            double n1 = Math.Sqrt(r10 * r10 + r11 * r11 + r12 * r12);
            if (n1 < 1e-9 || double.IsNaN(n1))
            {
                if (Math.Abs(r00) < 0.9) { r10 = r02; r11 = 0; r12 = -r00; }
                else { r10 = 0; r11 = -r02; r12 = r01; }
                n1 = Math.Sqrt(r10 * r10 + r11 * r11 + r12 * r12);
                r10 /= n1; r11 /= n1; r12 /= n1;
            }
            else
            {
                r10 /= n1; r11 /= n1; r12 /= n1;
            }
            double r20n = r01 * r12 - r02 * r11;
            double r21n = r02 * r10 - r00 * r12;
            double r22n = r00 * r11 - r01 * r10;
            p[0] = r00; p[1] = r01; p[2] = r02;
            p[4] = r10; p[5] = r11; p[6] = r12;
            p[8] = r20n; p[9] = r21n; p[10] = r22n;
            if (double.IsNaN(p[3]) || double.IsInfinity(p[3])) p[3] = 0;
            if (double.IsNaN(p[7]) || double.IsInfinity(p[7])) p[7] = 0;
            if (double.IsNaN(p[11]) || double.IsInfinity(p[11])) p[11] = 0;
            p[12] = 0; p[13] = 0; p[14] = 0; p[15] = 1;
        }

        static double[] MulT(double[] a, double[] b)
        {
            double[] r = new double[16];
            for (int c = 0; c < 4; c++)
            {
                for (int rw = 0; rw < 4; rw++)
                {
                    double s = 0;
                    for (int k = 0; k < 4; k++) s += a[rw * 4 + k] * b[k * 4 + c];
                    r[rw * 4 + c] = s;
                }
            }
            return r;
        }

        static double[] InvT(double[] t)
        {
            double[] r = new double[16];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    r[i * 4 + j] = t[j * 4 + i];
            for (int i = 0; i < 3; i++)
            {
                double s = 0;
                for (int k = 0; k < 3; k++) s += r[i * 4 + k] * t[k * 4 + 3];
                r[i * 4 + 3] = -s;
            }
            r[15] = 1;
            return r;
        }

        static void ApplyT(double[] t, float px, float py, float pz, out float ox, out float oy, out float oz)
        {
            ox = (float)(t[0] * px + t[1] * py + t[2] * pz + t[3]);
            oy = (float)(t[4] * px + t[5] * py + t[6] * pz + t[7]);
            oz = (float)(t[8] * px + t[9] * py + t[10] * pz + t[11]);
        }

        static void ApplyTRot(double[] t, float px, float py, float pz, out float ox, out float oy, out float oz)
        {
            ox = (float)(t[0] * px + t[1] * py + t[2] * pz);
            oy = (float)(t[4] * px + t[5] * py + t[6] * pz);
            oz = (float)(t[8] * px + t[9] * py + t[10] * pz);
        }

        static void ExpSO3(double wx, double wy, double wz, double[] r9)
        {
            double th = Math.Sqrt(wx * wx + wy * wy + wz * wz);
            if (th < 1e-12)
            {
                r9[0] = 1; r9[1] = -wz; r9[2] = wy;
                r9[3] = wz; r9[4] = 1; r9[5] = -wx;
                r9[6] = -wy; r9[7] = wx; r9[8] = 1;
                return;
            }
            double a = Math.Sin(th) / th;
            double b = (1 - Math.Cos(th)) / (th * th);
            double ux = wx / th, uy = wy / th, uz = wz / th;
            double c = 1 - a;
            r9[0] = 1 - b * (uy * uy + uz * uz);
            r9[1] = -a * uz + b * ux * uy;
            r9[2] = a * uy + b * ux * uz;
            r9[3] = a * uz + b * ux * uy;
            r9[4] = 1 - b * (ux * ux + uz * uz);
            r9[5] = -a * ux + b * uy * uz;
            r9[6] = -a * uy + b * ux * uz;
            r9[7] = a * ux + b * uy * uz;
            r9[8] = 1 - b * (ux * ux + uy * uy);
        }

        static void LogSO3(double[] r9, out double wx, out double wy, out double wz)
        {
            double tr = r9[0] + r9[4] + r9[8];
            double c = (tr - 1) * 0.5;
            if (c > 1) c = 1;
            if (c < -1) c = -1;
            double th = Math.Acos(c);
            if (th < 1e-10) { wx = 0; wy = 0; wz = 0; return; }
            double s = Math.Sin(th);
            if (Math.Abs(s) < 1e-10)
            {
                wx = (r9[0] - c); wy = (r9[4] - c); wz = (r9[8] - c);
                double n = Math.Sqrt(wx * wx + wy * wy + wz * wz);
                if (n < 1e-12) { wx = 0; wy = 0; wz = 0; return; }
                wx = wx / n * th; wy = wy / n * th; wz = wz / n * th;
                return;
            }
            double k = th / (2 * s);
            wx = (r9[7] - r9[5]) * k;
            wy = (r9[2] - r9[6]) * k;
            wz = (r9[3] - r9[1]) * k;
        }

        static double[] MulRotT(double[] pose, double[] r9)
        {
            double[] res = new double[16];
            Array.Copy(pose, res, 16);
            for (int rw = 0; rw < 3; rw++)
            {
                for (int c = 0; c < 3; c++)
                {
                    double s = 0;
                    for (int k = 0; k < 3; k++) s += r9[rw * 3 + k] * pose[k * 4 + c];
                    res[rw * 4 + c] = s;
                }
            }
            return res;
        }

        static long ModelKeyL(float x, float y, float z)
        {
            long ix = (long)Math.Floor(x / ModelCellSize) + 1048576;
            long iy = (long)Math.Floor(y / ModelCellSize) + 1048576;
            long iz = (long)Math.Floor(z / ModelCellSize) + 1048576;
            return (ix * 2097152L + iy) * 2097152L + iz;
        }

        static void ModelAddPoint(float x, float y, float z, float nx, float ny, float nz)
        {
            long key = ModelKeyL(x, y, z);
            int idx;
            if (!g_modelCells.TryGetValue(key, out idx))
            {
                if (g_modelList.Count >= MaxModelCells) return;
                ModelCell c = new ModelCell();
                c.x = x; c.y = y; c.z = z;
                c.nx = nx; c.ny = ny; c.nz = nz;
                c.cnt = 1;
                g_modelList.Add(c);
                g_modelCells[key] = g_modelList.Count - 1;
                return;
            }
            ModelCell cell = g_modelList[idx];
            float a = 1f / (cell.cnt + 1);
            cell.x += (x - cell.x) * a;
            cell.y += (y - cell.y) * a;
            cell.z += (z - cell.z) * a;
            cell.nx += (nx - cell.nx) * a;
            cell.ny += (ny - cell.ny) * a;
            cell.nz += (nz - cell.nz) * a;
            cell.cnt++;
        }

        static void RebuildModel()
        {
            g_modelCells.Clear();
            g_modelList.Clear();
            for (int k = 0; k < g_kfs.Count; k++)
            {
                ScanKf kf = g_kfs[k];
                for (int i = 0; i < kf.n; i++)
                {
                    float px = kf.pts[i * 3], py = kf.pts[i * 3 + 1], pz = kf.pts[i * 3 + 2];
                    float wx, wy, wz;
                    ApplyT(kf.pose, px, py, pz, out wx, out wy, out wz);
                    float rx, ry, rz;
                    ApplyTRot(kf.pose, kf.nrm[i * 3], kf.nrm[i * 3 + 1], kf.nrm[i * 3 + 2], out rx, out ry, out rz);
                    ModelAddPoint(wx, wy, wz, rx, ry, rz);
                }
            }
        }

        static bool Solve6(double[,] A, double[] b, double[] x)
        {
            int n = 6;
            double[,] m = new double[6, 12];
            for (int i = 0; i < 6; i++)
            {
                for (int j = 0; j < 6; j++) m[i, j] = A[i, j];
                for (int j = 0; j < 6; j++) m[i, 6 + j] = (i == j) ? 1 : 0;
            }
            for (int col = 0; col < 6; col++)
            {
                int piv = col;
                double best = Math.Abs(m[col, col]);
                for (int r2 = col + 1; r2 < 6; r2++)
                {
                    double v = Math.Abs(m[r2, col]);
                    if (v > best) { best = v; piv = r2; }
                }
                if (best < 1e-14) return false;
                if (piv != col)
                {
                    for (int c2 = 0; c2 < 12; c2++)
                    {
                        double tmp = m[col, c2];
                        m[col, c2] = m[piv, c2];
                        m[piv, c2] = tmp;
                    }
                }
                double d = m[col, col];
                for (int c2 = 0; c2 < 12; c2++) m[col, c2] /= d;
                for (int r2 = 0; r2 < 6; r2++)
                {
                    if (r2 == col) continue;
                    double f = m[r2, col];
                    if (f == 0) continue;
                    for (int c2 = 0; c2 < 12; c2++) m[r2, c2] -= f * m[col, c2];
                }
            }
            for (int i = 0; i < 6; i++) x[i] = m[i, 6];
            return true;
        }

        static bool RunIcp(float[] srcPts, float[] srcNrm, int srcN, double[] T, int iters, out int inliers, out float rmse)
        {
            inliers = 0;
            rmse = 1e9f;
            if (g_modelList.Count == 0 || srcN < 50) return false;
            double[] pose = (double[])T.Clone();
            double[] upd = new double[6];
            for (int it = 0; it < iters; it++)
            {
                double[,] A = new double[6, 6];
                double[] b = new double[6];
                double sumE2 = 0;
                int cnt = 0;
                for (int i = 0; i < srcN; i++)
                {
                    float px, py, pz;
                    ApplyT(pose, srcPts[i * 3], srcPts[i * 3 + 1], srcPts[i * 3 + 2], out px, out py, out pz);
                    float rx, ry, rz;
                    ApplyTRot(pose, srcNrm[i * 3], srcNrm[i * 3 + 1], srcNrm[i * 3 + 2], out rx, out ry, out rz);
                    float cn = (float)Math.Sqrt(rx * rx + ry * ry + rz * rz);
                    if (cn < 1e-6f) continue;
                    rx /= cn; ry /= cn; rz /= cn;

                    long baseKey = ModelKeyL(px, py, pz);
                    long ix = (long)Math.Floor(px / ModelCellSize) + 1048576;
                    long iy = (long)Math.Floor(py / ModelCellSize) + 1048576;
                    long iz = (long)Math.Floor(pz / ModelCellSize) + 1048576;
                    int bestIdx = -1;
                    float bestD2 = IcpCorr * IcpCorr;
                    for (int dx = -1; dx <= 1; dx++)
                        for (int dy = -1; dy <= 1; dy++)
                            for (int dz = -1; dz <= 1; dz++)
                            {
                                long key = ((ix + dx) * 2097152L + (iy + dy)) * 2097152L + (iz + dz);
                                int idx;
                                if (!g_modelCells.TryGetValue(key, out idx)) continue;
                                ModelCell c = g_modelList[idx];
                                float ex = px - c.x, ey = py - c.y, ez = pz - c.z;
                                float d2 = ex * ex + ey * ey + ez * ez;
                                if (d2 < bestD2) { bestD2 = d2; bestIdx = idx; }
                            }
                    if (bestIdx < 0) continue;
                    ModelCell mc = g_modelList[bestIdx];
                    float e = rx * (px - mc.x) + ry * (py - mc.y) + rz * (pz - mc.z);
                    double[] J = new double[6];
                    J[0] = rx; J[1] = ry; J[2] = rz;
                    J[3] = py * rz - pz * ry;
                    J[4] = pz * rx - px * rz;
                    J[5] = px * ry - py * rx;
                    for (int a = 0; a < 6; a++)
                    {
                        for (int bq = 0; bq < 6; bq++) A[a, bq] += J[a] * J[bq];
                        b[a] -= J[a] * e;
                    }
                    sumE2 += (double)e * e;
                    cnt++;
                }
                if (cnt < 40)
                {
                    if (inliers >= 40) break;
                    return false;
                }
                inliers = cnt;
                rmse = (float)Math.Sqrt(sumE2 / cnt);
                double maxD = 0;
                for (int a = 0; a < 6; a++) if (A[a, a] > maxD) maxD = A[a, a];
                if (maxD < 1e-12) maxD = 1;
                for (int a = 0; a < 6; a++) A[a, a] += maxD * 1e-6 + 1e-6;
                if (!Solve6(A, b, upd)) break;
                double un = Math.Sqrt(upd[0] * upd[0] + upd[1] * upd[1] + upd[2] * upd[2] +
                                      upd[3] * upd[3] + upd[4] * upd[4] + upd[5] * upd[5]);
                double rotN = Math.Sqrt(upd[3] * upd[3] + upd[4] * upd[4] + upd[5] * upd[5]);
                double trN = Math.Sqrt(upd[0] * upd[0] + upd[1] * upd[1] + upd[2] * upd[2]);
                if (double.IsNaN(un) || trN > 300 || rotN > 0.6) break;
                double dwx = upd[3], dwy = upd[4], dwz = upd[5];
                double[] R = new double[9];
                ExpSO3(dwx, dwy, dwz, R);
                double[] newPose = new double[16];
                Array.Copy(pose, newPose, 16);
                for (int rw = 0; rw < 3; rw++)
                {
                    for (int c = 0; c < 3; c++)
                    {
                        double s = 0;
                        for (int k = 0; k < 3; k++) s += R[rw * 3 + k] * pose[k * 4 + c];
                        newPose[rw * 4 + c] = s;
                    }
                }
                double t0 = pose[3], t1 = pose[7], t2 = pose[11];
                newPose[3] = R[0] * t0 + R[1] * t1 + R[2] * t2 + upd[0];
                newPose[7] = R[3] * t0 + R[4] * t1 + R[5] * t2 + upd[1];
                newPose[11] = R[6] * t0 + R[7] * t1 + R[8] * t2 + upd[2];
                pose = newPose;
                SanitizePose(pose);
                if (un < 1e-4) break;
            }
            T[0] = pose[0]; T[1] = pose[1]; T[2] = pose[2]; T[3] = pose[3];
            T[4] = pose[4]; T[5] = pose[5]; T[6] = pose[6]; T[7] = pose[7];
            T[8] = pose[8]; T[9] = pose[9]; T[10] = pose[10]; T[11] = pose[11];
            T[12] = 0; T[13] = 0; T[14] = 0; T[15] = 1;
            SanitizePose(T);
            return inliers >= 40;
        }

        static bool RunIcpTarget(float[] srcPts, float[] srcNrm, int srcN, float[] tgtPts, float[] tgtNrm, int tgtN, double[] tgtPose, double[] T, out int inliers, out float rmse)
        {
            inliers = 0;
            rmse = 1e9f;
            Dictionary<long, int> savedCells = g_modelCells;
            List<ModelCell> savedList = g_modelList;
            g_modelCells = new Dictionary<long, int>();
            g_modelList = new List<ModelCell>();
            try
            {
                for (int i = 0; i < tgtN; i++)
                {
                    float wx, wy, wz;
                    ApplyT(tgtPose, tgtPts[i * 3], tgtPts[i * 3 + 1], tgtPts[i * 3 + 2], out wx, out wy, out wz);
                    float rx, ry, rz;
                    ApplyTRot(tgtPose, tgtNrm[i * 3], tgtNrm[i * 3 + 1], tgtNrm[i * 3 + 2], out rx, out ry, out rz);
                    ModelAddPoint(wx, wy, wz, rx, ry, rz);
                }
                bool ok = RunIcp(srcPts, srcNrm, srcN, T, 12, out inliers, out rmse);
                return ok;
            }
            finally
            {
                g_modelCells = savedCells;
                g_modelList = savedList;
            }
        }

        static void BuildSrc(ushort[] depth, int stride, out float[] pts, out float[] nrm, out int count)
        {
            int maxN = ((W / stride) + 2) * ((H / stride) + 2);
            pts = new float[maxN * 3];
            nrm = new float[maxN * 3];
            count = 0;
            float fx = g_dFx, fy = g_dFy;
            float cx = W / 2f, cy = H / 2f;
            int r2 = stride * 2;
            for (int v = r2; v < H - r2; v += stride)
            {
                int row = v * W;
                for (int u = r2; u < W - r2; u += stride)
                {
                    int d = depth[row + u];
                    if (d < 300 || d > 6000) continue;
                    int dl = depth[row + u - r2];
                    int dr = depth[row + u + r2];
                    int dd = depth[row + r2 * W + u];
                    int du = depth[(v - r2) * W + u];
                    if (dl < 300 || dr < 300 || du < 300 || dd < 300) continue;
                    if (Math.Abs(d - dl) > 80 || Math.Abs(d - dr) > 80 ||
                        Math.Abs(d - du) > 80 || Math.Abs(d - dd) > 80) continue;
                    float z = d;
                    float x = (u - cx) * z / fx;
                    float y = (v - cy) * z / fy;
                    float x1 = (u + r2 - cx) * dr / fx;
                    float y1 = (v - cy) * dr / fy;
                    float z1 = dr;
                    float x2 = (u - r2 - cx) * dl / fx;
                    float y2 = (v - cy) * dl / fy;
                    float z2 = dl;
                    float ax = x1 - x2, ay = y1 - y2, az = z1 - z2;
                    float x3 = x, y3 = (v + r2 - cy) * dd / fy, z3 = dd;
                    float x4 = x, y4 = (v - r2 - cy) * du / fy, z4 = du;
                    float bx = x3 - x4, by = y3 - y4, bz = z3 - z4;
                    float nx = ay * bz - az * by;
                    float ny = az * bx - ax * bz;
                    float nz = ax * by - ay * bx;
                    float nl = (float)Math.Sqrt(nx * nx + ny * ny + nz * nz);
                    if (nl < 1e-3f) continue;
                    nx /= nl; ny /= nl; nz /= nl;
                    if (nx * x + ny * y + nz * z > 0) { nx = -nx; ny = -ny; nz = -nz; }
                    int i3 = count * 3;
                    pts[i3] = x; pts[i3 + 1] = y; pts[i3 + 2] = z;
                    nrm[i3] = nx; nrm[i3 + 1] = ny; nrm[i3 + 2] = nz;
                    count++;
                }
            }
        }

        static float[] BuildDesc(float[] pts, int n)
        {
            float[] bins = new float[128];
            int[] cnts = new int[128];
            for (int i = 0; i < n; i++)
            {
                float x = pts[i * 3], y = pts[i * 3 + 1], z = pts[i * 3 + 2];
                if (z < 300 || z > 6000) continue;
                double az = Math.Atan2(x, z);
                double el = Math.Atan2(y, Math.Sqrt(x * x + z * z));
                int ai = (int)((az + Math.PI) / (2 * Math.PI) * 32);
                if (ai < 0) ai = 0; if (ai > 31) ai = 31;
                int ei = (int)((el + 0.7) / 1.4 * 4);
                if (ei < 0) ei = 0; if (ei > 3) ei = 3;
                int b = ei * 32 + ai;
                bins[b] += z;
                cnts[b]++;
            }
            float mean = 0;
            int mc = 0;
            for (int i = 0; i < 128; i++)
            {
                if (cnts[i] > 0) { bins[i] /= cnts[i]; mean += bins[i]; mc++; }
            }
            if (mc > 0)
            {
                mean /= mc;
                for (int i = 0; i < 128; i++) if (cnts[i] > 0) bins[i] /= mean;
            }
            return bins;
        }

        static float DescSim(float[] a, float[] b)
        {
            double dot = 0, na = 0, nb = 0;
            for (int i = 0; i < 128; i++)
            {
                dot += (double)a[i] * b[i];
                na += (double)a[i] * a[i];
                nb += (double)b[i] * b[i];
            }
            if (na < 1e-9 || nb < 1e-9) return 0;
            return (float)(dot / (Math.Sqrt(na) * Math.Sqrt(nb)));
        }

        static void ScanResetLocked()
        {
            g_pose = IdentT();
            g_lastPose = IdentT();
            g_prevPose = IdentT();
            g_trackState = StateOff;
            g_trackInliers = 0;
            g_trackRmse = 0;
            g_trackSeq = 0;
            g_kfBootstrapped = false;
            g_kfs.Clear();
            g_loops.Clear();
            g_modelCells.Clear();
            g_modelList.Clear();
            g_deltas = new double[0];
            g_deltaOptSeq = -1;
            g_optSeq = 0;
            g_icpMainOk = 0;
            g_icpMainFail = 0;
        }

        static void ScanReset()
        {
            lock (g_trackLock) { ScanResetLocked(); }
        }

        static bool ScanTrackSet(bool on)
        {
            lock (g_trackLock)
            {
                if (on && !g_trackOn)
                {
                    ScanResetLocked();
                    g_trackOn = true;
                    g_trackState = StateWeak;
                    Console.WriteLine("Scan tracking ON");
                }
                else if (!on && g_trackOn)
                {
                    g_trackOn = false;
                    g_trackState = StateOff;
                    Console.WriteLine("Scan tracking OFF");
                }
            }
            g_trackPulse.Set();
            return true;
        }

        static void AddKeyframe(float[] pts, float[] nrm, int n, double[] pose, double[] measFromPrev)
        {
            ScanKf kf = new ScanKf();
            int stride = 2;
            int kn = (n + stride - 1) / stride;
            kf.pts = new float[kn * 3];
            kf.nrm = new float[kn * 3];
            int w = 0;
            for (int i = 0; i < n; i += stride)
            {
                kf.pts[w * 3] = pts[i * 3];
                kf.pts[w * 3 + 1] = pts[i * 3 + 1];
                kf.pts[w * 3 + 2] = pts[i * 3 + 2];
                kf.nrm[w * 3] = nrm[i * 3];
                kf.nrm[w * 3 + 1] = nrm[i * 3 + 1];
                kf.nrm[w * 3 + 2] = nrm[i * 3 + 2];
                w++;
            }
            kf.n = w;
            kf.pose = (double[])pose.Clone();
            if (measFromPrev != null) kf.meas = (double[])measFromPrev.Clone();
            kf.desc = BuildDesc(pts, n);
            g_kfs.Add(kf);
        }

        static bool ShouldMakeKf()
        {
            if (g_kfs.Count == 0) return true;
            ScanKf last = g_kfs[g_kfs.Count - 1];
            double dx = g_pose[3] - last.pose[3];
            double dy = g_pose[7] - last.pose[7];
            double dz = g_pose[11] - last.pose[11];
            double dist = Math.Sqrt(dx * dx + dy * dy + dz * dz);
            if (dist > 150) return true;
            double[] rel = MulT(InvT(last.pose), g_pose);
            double wx, wy, wz;
            LogSO3(rel, out wx, out wy, out wz);
            double ang = Math.Sqrt(wx * wx + wy * wy + wz * wz) * 180.0 / Math.PI;
            if (ang > 10) return true;
            return false;
        }

        static void DetectLoops(int curIdx, int maxJ)
        {
            int n = g_kfs.Count;
            if (n < 12 || curIdx < 11 || curIdx >= n) return;
            ScanKf cur = g_kfs[curIdx];
            List<KeyValuePair<double, int>> cands = new List<KeyValuePair<double, int>>();
            for (int j = 0; j <= maxJ && j < curIdx - 10; j++)
            {
                ScanKf other = g_kfs[j];
                float sim = DescSim(cur.desc, other.desc);
                double dx = cur.pose[3] - other.pose[3];
                double dy = cur.pose[7] - other.pose[7];
                double dz = cur.pose[11] - other.pose[11];
                double dist = Math.Sqrt(dx * dx + dy * dy + dz * dz);
                bool sel = (sim > 0.70f) || (dist < 1500 && sim > 0.55f);
                if (!sel) continue;
                double score = sim + (dist < 1500 ? 0.15 : 0);
                cands.Add(new KeyValuePair<double, int>(score, j));
            }
            cands.Sort(delegate(KeyValuePair<double, int> a, KeyValuePair<double, int> b)
            {
                return b.Key.CompareTo(a.Key);
            });
            int tries = 0;
            for (int ci = 0; ci < cands.Count && tries < 6; ci++)
            {
                int j = cands[ci].Value;
                tries++;
                ScanKf other = g_kfs[j];
                double[] guess = (double[])other.pose.Clone();
                int inl;
                float rms;
                bool ok = RunIcpTarget(cur.pts, cur.nrm, cur.n, other.pts, other.nrm, other.n,
                    other.pose, guess, out inl, out rms);
                if (!ok || inl < 200 || rms > 60f) continue;
                bool dup = false;
                for (int li = 0; li < g_loops.Count; li++)
                {
                    if (g_loops[li].j == curIdx && Math.Abs(g_loops[li].i - j) <= 2) { dup = true; break; }
                }
                if (dup) continue;
                LoopEdge e = new LoopEdge();
                e.i = j;
                e.j = curIdx;
                e.z = MulT(InvT(other.pose), guess);
                g_loops.Add(e);
                Console.WriteLine("Loop closure: kf " + j + " <-> " + curIdx + " inliers=" + inl + " rmse=" + rms.ToString("0.0"));
                RunPoseGraph();
                return;
            }
        }

        static void EdgeResidual(double[] ti, double[] tj, double[] z, double[] r)
        {
            double[] pred = MulT(InvT(ti), tj);
            double[] e = MulT(InvT(z), pred);
            double wx, wy, wz;
            LogSO3(e, out wx, out wy, out wz);
            r[0] = wx; r[1] = wy; r[2] = wz;
            r[3] = e[3]; r[4] = e[7]; r[5] = e[11];
        }

        static double[] PerturbNode(double[] pose, int a, double eps)
        {
            double[] outPose = (double[])pose.Clone();
            if (a < 3)
            {
                double[] w = new double[9];
                double ex = 0, ey = 0, ez = 0;
                if (a == 0) ex = eps;
                else if (a == 1) ey = eps;
                else ez = eps;
                ExpSO3(ex, ey, ez, w);
                for (int rw = 0; rw < 3; rw++)
                    for (int c = 0; c < 3; c++)
                    {
                        double s = 0;
                        for (int k = 0; k < 3; k++) s += w[rw * 3 + k] * pose[k * 4 + c];
                        outPose[rw * 4 + c] = s;
                    }
            }
            else
            {
                outPose[3 + (a - 3) * 4] = pose[3 + (a - 3) * 4] + eps;
            }
            return outPose;
        }

        static void HMatVec(int eCount, int[] eI, int[] eJ, double[][] JiArr, double[][] JjArr, double[] v, double[] hv)
        {
            for (int i = 0; i < hv.Length; i++) hv[i] = 0;
            double[] w = new double[6];
            for (int e = 0; e < eCount; e++)
            {
                double[] Ji = JiArr[e];
                double[] Jj = JjArr[e];
                int ia = eI[e], ja = eJ[e];
                int oi = ia > 0 ? (ia - 1) * 6 : -1;
                int oj = ja > 0 ? (ja - 1) * 6 : -1;
                for (int k = 0; k < 6; k++) w[k] = 0;
                if (Ji != null)
                    for (int a = 0; a < 6; a++)
                        for (int k = 0; k < 6; k++)
                            w[k] += Ji[k * 6 + a] * v[oi + a];
                if (Jj != null)
                    for (int a = 0; a < 6; a++)
                        for (int k = 0; k < 6; k++)
                            w[k] += Jj[k * 6 + a] * v[oj + a];
                if (Ji != null)
                    for (int a = 0; a < 6; a++)
                        for (int k = 0; k < 6; k++)
                            hv[oi + a] += Ji[k * 6 + a] * w[k];
                if (Jj != null)
                    for (int a = 0; a < 6; a++)
                        for (int k = 0; k < 6; k++)
                            hv[oj + a] += Jj[k * 6 + a] * w[k];
            }
        }

        static void Pcg(int eCount, int[] eI, int[] eJ, double[][] JiArr, double[][] JjArr,
            double[] diag, double[] b, double[] x)
        {
            int n = b.Length;
            for (int i = 0; i < n; i++) x[i] = 0;
            double[] r = new double[n];
            double[] z = new double[n];
            double[] p = new double[n];
            double[] hp = new double[n];
            Array.Copy(b, r, n);
            double bnorm = 0;
            for (int i = 0; i < n; i++) bnorm += r[i] * r[i];
            bnorm = Math.Sqrt(bnorm);
            if (bnorm < 1e-14) return;
            for (int i = 0; i < n; i++) z[i] = r[i] / diag[i];
            Array.Copy(z, p, n);
            double rz = 0;
            for (int i = 0; i < n; i++) rz += r[i] * z[i];
            for (int it = 0; it < 250; it++)
            {
                HMatVec(eCount, eI, eJ, JiArr, JjArr, p, hp);
                double php = 0;
                for (int i = 0; i < n; i++) php += p[i] * hp[i];
                if (Math.Abs(php) < 1e-18) break;
                double alpha = rz / php;
                double rnorm = 0;
                for (int i = 0; i < n; i++)
                {
                    x[i] += alpha * p[i];
                    r[i] -= alpha * hp[i];
                    rnorm += r[i] * r[i];
                }
                rnorm = Math.Sqrt(rnorm);
                if (rnorm < 1e-8 * bnorm) break;
                for (int i = 0; i < n; i++) z[i] = r[i] / diag[i];
                double rzNew = 0;
                for (int i = 0; i < n; i++) rzNew += r[i] * z[i];
                double beta = rzNew / rz;
                rz = rzNew;
                for (int i = 0; i < n; i++) p[i] = z[i] + beta * p[i];
            }
        }

        static void RunPoseGraph()
        {
            int n = g_kfs.Count;
            if (n < 2) return;
            if (g_loops.Count == 0) return;
            double[][] oldPoses = new double[n][];
            double[][] nodes = new double[n][];
            for (int i = 0; i < n; i++)
            {
                oldPoses[i] = (double[])g_kfs[i].pose.Clone();
                nodes[i] = (double[])g_kfs[i].pose.Clone();
            }
            int eCount = (n - 1) + g_loops.Count;
            int[] eI = new int[eCount];
            int[] eJ = new int[eCount];
            double[] ezAll = new double[eCount * 16];
            int ei2 = 0;
            for (int k = 1; k < n; k++)
            {
                eI[ei2] = k - 1;
                eJ[ei2] = k;
                for (int a = 0; a < 16; a++) ezAll[ei2 * 16 + a] = g_kfs[k].meas[a];
                ei2++;
            }
            for (int l = 0; l < g_loops.Count; l++)
            {
                eI[ei2] = g_loops[l].i;
                eJ[ei2] = g_loops[l].j;
                for (int a = 0; a < 16; a++) ezAll[ei2 * 16 + a] = g_loops[l].z[a];
                ei2++;
            }
            int vars = (n - 1) * 6;
            double[] x = new double[vars];
            double epsR = 1e-5;
            double epsT = 1e-2;
            double[] zEdge = new double[16];
            for (int iter = 0; iter < 8; iter++)
            {
                double[] g = new double[vars];
                double[] diag = new double[vars];
                double[][] JiArr = new double[eCount][];
                double[][] JjArr = new double[eCount][];
                double[] r = new double[6];
                double[] rp = new double[6];
                for (int e = 0; e < eCount; e++)
                {
                    int ia = eI[e], ja = eJ[e];
                    for (int a = 0; a < 16; a++) zEdge[a] = ezAll[e * 16 + a];
                    EdgeResidual(nodes[ia], nodes[ja], zEdge, r);
                    double[] Ji = null;
                    double[] Jj = null;
                    if (ia > 0)
                    {
                        Ji = new double[36];
                        for (int a = 0; a < 6; a++)
                        {
                            double[] save = nodes[ia];
                            nodes[ia] = PerturbNode(save, a, a < 3 ? epsR : epsT);
                            EdgeResidual(nodes[ia], nodes[ja], zEdge, rp);
                            nodes[ia] = save;
                            for (int k = 0; k < 6; k++) Ji[k * 6 + a] = (rp[k] - r[k]) / (a < 3 ? epsR : epsT);
                        }
                    }
                    if (ja > 0)
                    {
                        Jj = new double[36];
                        for (int a = 0; a < 6; a++)
                        {
                            double[] save = nodes[ja];
                            nodes[ja] = PerturbNode(save, a, a < 3 ? epsR : epsT);
                            EdgeResidual(nodes[ia], nodes[ja], zEdge, rp);
                            nodes[ja] = save;
                            for (int k = 0; k < 6; k++) Jj[k * 6 + a] = (rp[k] - r[k]) / (a < 3 ? epsR : epsT);
                        }
                    }
                    JiArr[e] = Ji;
                    JjArr[e] = Jj;
                    int oi = ia > 0 ? (ia - 1) * 6 : -1;
                    int oj = ja > 0 ? (ja - 1) * 6 : -1;
                    if (Ji != null)
                        for (int a = 0; a < 6; a++)
                        {
                            double s = 0;
                            for (int k = 0; k < 6; k++) s += Ji[k * 6 + a] * r[k];
                            g[oi + a] += s;
                            double ds = 0;
                            for (int k = 0; k < 6; k++) ds += Ji[k * 6 + a] * Ji[k * 6 + a];
                            diag[oi + a] += ds;
                        }
                    if (Jj != null)
                        for (int a = 0; a < 6; a++)
                        {
                            double s = 0;
                            for (int k = 0; k < 6; k++) s += Jj[k * 6 + a] * r[k];
                            g[oj + a] += s;
                            double ds = 0;
                            for (int k = 0; k < 6; k++) ds += Jj[k * 6 + a] * Jj[k * 6 + a];
                            diag[oj + a] += ds;
                        }
                }
                for (int i = 0; i < vars; i++)
                {
                    diag[i] += 1e-4;
                    if (diag[i] < 1e-8) diag[i] = 1e-8;
                    x[i] = -g[i];
                }
                double[] rhs = new double[vars];
                for (int i = 0; i < vars; i++) rhs[i] = -g[i];
                Pcg(eCount, eI, eJ, JiArr, JjArr, diag, rhs, x);
                double step = 0;
                for (int k = 1; k < n; k++)
                {
                    int o = (k - 1) * 6;
                    double[] w9 = new double[9];
                    ExpSO3(x[o], x[o + 1], x[o + 2], w9);
                    double[] res = new double[16];
                    Array.Copy(nodes[k], res, 16);
                    for (int rw = 0; rw < 3; rw++)
                        for (int c = 0; c < 3; c++)
                        {
                            double s = 0;
                            for (int kk = 0; kk < 3; kk++) s += w9[rw * 3 + kk] * nodes[k][kk * 4 + c];
                            res[rw * 4 + c] = s;
                        }
                    res[3] += x[o + 3];
                    res[7] += x[o + 4];
                    res[11] += x[o + 5];
                    nodes[k] = res;
                    step += Math.Abs(x[o]) + Math.Abs(x[o + 1]) + Math.Abs(x[o + 2]) +
                            Math.Abs(x[o + 3]) + Math.Abs(x[o + 4]) + Math.Abs(x[o + 5]);
                }
                if (step < 1e-6) break;
            }
            g_deltas = new double[n * 16];
            for (int i = 0; i < n; i++)
            {
                double[] d = MulT(nodes[i], InvT(oldPoses[i]));
                for (int a = 0; a < 16; a++) g_deltas[i * 16 + a] = d[a];
                g_kfs[i].pose = nodes[i];
            }
            double[] dLast = new double[16];
            Array.Copy(g_deltas, (n - 1) * 16, dLast, 0, 16);
            g_lastPose = MulT(dLast, g_lastPose);
            g_prevPose = (double[])g_lastPose.Clone();
            g_pose = (double[])g_lastPose.Clone();
            g_optSeq++;
            g_deltaOptSeq = g_optSeq;
            RebuildModel();
            Console.WriteLine("Pose graph ottimizzato: kf=" + n + " loops=" + g_loops.Count + " optSeq=" + g_optSeq);
        }

        static void TrackLoop()
        {
            while (true)
            {
                g_trackPulse.WaitOne(120);
                if (!g_trackOn) continue;
                ushort[] snap = new ushort[N];
                try
                {
                    lock (g_depthLock) { Buffer.BlockCopy(g_depth, 0, snap, 0, N * 2); }
                }
                catch { continue; }
                float[] pts, nrm;
                int cnt;
                try { BuildSrc(snap, 4, out pts, out nrm, out cnt); }
                catch { continue; }
                lock (g_trackLock)
                {
                    if (!g_trackOn) continue;
                    try
                    {
                        if (!g_kfBootstrapped)
                        {
                            if (cnt < 80) { g_trackState = StateWeak; continue; }
                            g_pose = IdentT();
                            g_lastPose = IdentT();
                            g_prevPose = IdentT();
                            double[] meas = IdentT();
                            AddKeyframe(pts, nrm, cnt, g_pose, meas);
                            RebuildModel();
                            g_kfBootstrapped = true;
                            g_trackState = StateOk;
                            g_trackInliers = cnt;
                            g_trackRmse = 0;
                            g_trackSeq++;
                            continue;
                        }
                        if (cnt < 80)
                        {
                            g_trackState = StateLost;
                            g_trackSeq++;
                            continue;
                        }
                        double[] guess;
                        double[] rel = MulT(InvT(g_prevPose), g_lastPose);
                        guess = MulT(g_lastPose, rel);
                        SanitizePose(guess);
                        SanitizePose(g_lastPose);
                        SanitizePose(g_prevPose);
                        int inl = 0;
                        float rms = 1e9f;
                        bool ok = RunIcp(pts, nrm, cnt, guess, 10, out inl, out rms);
                        if (!ok || inl < 150 || rms > 70f)
                        {
                            double[] alt = IdentT();
                            int inlAlt = 0;
                            float rmsAlt = 1e9f;
                            if (RunIcp(pts, nrm, cnt, alt, 10, out inlAlt, out rmsAlt) &&
                                inlAlt >= 150 && rmsAlt <= 70f)
                            {
                                guess = alt;
                                ok = true;
                                inl = inlAlt;
                                rms = rmsAlt;
                            }
                        }
                        g_icpMainOk += ok ? 1 : 0;
                        g_icpMainFail += ok ? 0 : 1;
                        if (!ok || inl < 150 || rms > 70f)
                        {
                            if (g_icpMainFail % 30 == 1)
                            {
                                float bx0 = 1e9f, by0 = 1e9f, bz0 = 1e9f, bx1 = -1e9f, by1 = -1e9f, bz1 = -1e9f;
                                for (int mi = 0; mi < g_modelList.Count; mi++)
                                {
                                    ModelCell mc2 = g_modelList[mi];
                                    if (mc2.x < bx0) bx0 = mc2.x; if (mc2.x > bx1) bx1 = mc2.x;
                                    if (mc2.y < by0) by0 = mc2.y; if (mc2.y > by1) by1 = mc2.y;
                                    if (mc2.z < bz0) bz0 = mc2.z; if (mc2.z > bz1) bz1 = mc2.z;
                                }
                                float sp0 = 0, sp1 = 0, sp2 = 0;
                                if (cnt > 0)
                                {
                                    ApplyT(guess, pts[0], pts[1], pts[2], out sp0, out sp1, out sp2);
                                }
                                Console.WriteLine("ICP fail: ok=" + ok + " inl=" + inl + " rms=" +
                                    rms.ToString("0.0") + " srcN=" + cnt + " model=" + g_modelList.Count +
                                    " guessT=(" + guess[3].ToString("0") + "," + guess[7].ToString("0") +
                                    "," + guess[11].ToString("0") + ") guessR=(" +
                                    guess[0].ToString("0.000") + "," + guess[1].ToString("0.000") + "," +
                                    guess[2].ToString("0.000") + ";" + guess[4].ToString("0.000") + "," +
                                    guess[5].ToString("0.000") + "," + guess[6].ToString("0.000") + ";" +
                                    guess[8].ToString("0.000") + "," + guess[9].ToString("0.000") + "," +
                                    guess[10].ToString("0.000") + ") rawP0=(" +
                                    (cnt > 0 ? pts[0].ToString("0") : "?") + "," +
                                    (cnt > 0 ? pts[1].ToString("0") : "?") + "," +
                                    (cnt > 0 ? pts[2].ToString("0") : "?") + ") p0=(" +
                                    sp0.ToString("0") + "," + sp1.ToString("0") + "," + sp2.ToString("0") +
                                    ") mainOk=" + g_icpMainOk + " mainFail=" + g_icpMainFail);
                            }
                            double[] bestPose = null;
                            int bestInl = 0;
                            float bestRms = 1e9f;
                            int tries = 0;
                            for (int k = g_kfs.Count - 1; k >= 0 && tries < 8; k--, tries++)
                            {
                                ScanKf kf = g_kfs[k];
                                double[] g2 = (double[])kf.pose.Clone();
                                int inl2;
                                float rms2;
                                if (RunIcpTarget(pts, nrm, cnt, kf.pts, kf.nrm, kf.n, kf.pose, g2, out inl2, out rms2))
                                {
                                    if (inl2 > bestInl || (inl2 == bestInl && rms2 < bestRms))
                                    {
                                        bestInl = inl2;
                                        bestRms = rms2;
                                        bestPose = g2;
                                    }
                                }
                            }
                            if (bestPose != null && bestInl >= 250 && bestRms <= 60f)
                            {
                                SanitizePose(bestPose);
                                g_prevPose = (double[])bestPose.Clone();
                                g_lastPose = (double[])bestPose.Clone();
                                g_pose = (double[])bestPose.Clone();
                                g_trackState = StateOk;
                                g_trackInliers = bestInl;
                                g_trackRmse = bestRms;
                                g_trackSeq++;
                                Console.WriteLine("Relocalization ok: inliers=" + bestInl);
                                continue;
                            }
                            g_trackState = StateLost;
                            g_trackSeq++;
                            continue;
                        }
                        SanitizePose(guess);
                        g_prevPose = (double[])g_lastPose.Clone();
                        g_lastPose = (double[])guess.Clone();
                        g_pose = (double[])guess.Clone();
                        g_trackInliers = inl;
                        g_trackRmse = rms;
                        g_trackState = (inl >= 400 && rms < 40f) ? StateOk : StateWeak;
                        g_trackSeq++;
                        if (g_kfs.Count < MaxKf && ShouldMakeKf())
                        {
                            ScanKf lastKf = g_kfs[g_kfs.Count - 1];
                            double[] meas = MulT(InvT(lastKf.pose), g_pose);
                            AddKeyframe(pts, nrm, cnt, g_pose, meas);
                            RebuildModel();
                            DetectLoops(g_kfs.Count - 1, g_kfs.Count - 12);
                        }
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine("Track: " + ex.Message);
                        g_trackState = StateLost;
                    }
                }
            }
        }

        static string ScanStateName(int s)
        {
            if (s == StateOk) return "ok";
            if (s == StateWeak) return "weak";
            if (s == StateLost) return "lost";
            return "off";
        }

        static string BuildScanFields()
        {
            int st, inl, seq, kf, loops, opt;
            float rms;
            bool on;
            lock (g_trackLock)
            {
                st = g_trackState;
                inl = g_trackInliers;
                rms = g_trackRmse;
                seq = (int)g_trackSeq;
                kf = g_kfs.Count;
                loops = g_loops.Count;
                opt = g_optSeq;
                on = g_trackOn;
            }
            StringBuilder sb = new StringBuilder(256);
            sb.Append("\"track\":").Append(on ? "true" : "false");
            sb.Append(",\"state\":\"").Append(ScanStateName(st)).Append("\"");
            sb.Append(",\"inliers\":").Append(inl);
            sb.Append(",\"rmse\":").Append(D1(rms));
            sb.Append(",\"seq\":").Append(seq);
            sb.Append(",\"kf\":").Append(kf);
            sb.Append(",\"loops\":").Append(loops);
            sb.Append(",\"optSeq\":").Append(opt);
            return sb.ToString();
        }

        static byte[] BuildDepthWithPose()
        {
            byte[] depth = new byte[N * 2];
            int st, inl, kf, loops, opt;
            float rms;
            long seq;
            double[] pose;
            lock (g_trackLock)
            {
                st = g_trackState;
                inl = g_trackInliers;
                rms = g_trackRmse;
                seq = g_trackSeq;
                kf = g_kfs.Count;
                loops = g_loops.Count;
                opt = g_optSeq;
                pose = (double[])g_pose.Clone();
            }
            lock (g_depthLock) { Buffer.BlockCopy(g_depth, 0, depth, 0, N * 2); }
            byte[] outb = new byte[PoseHeaderBytes + N * 2];
            outb[0] = (byte)'P';
            outb[1] = (byte)'O';
            outb[2] = (byte)'S';
            outb[3] = (byte)'E';
            BitConverter.GetBytes((uint)seq).CopyTo(outb, 4);
            BitConverter.GetBytes((uint)st).CopyTo(outb, 8);
            for (int i = 0; i < 16; i++)
                BitConverter.GetBytes((float)pose[i]).CopyTo(outb, 12 + i * 4);
            BitConverter.GetBytes(rms).CopyTo(outb, 76);
            BitConverter.GetBytes(inl).CopyTo(outb, 80);
            BitConverter.GetBytes(kf).CopyTo(outb, 84);
            BitConverter.GetBytes(loops).CopyTo(outb, 88);
            BitConverter.GetBytes(opt).CopyTo(outb, 92);
            Buffer.BlockCopy(depth, 0, outb, PoseHeaderBytes, N * 2);
            return outb;
        }

        static string BuildDeltasJson(int reqOpt)
        {
            double[] d;
            int opt, n;
            lock (g_trackLock)
            {
                d = (double[])g_deltas.Clone();
                opt = g_deltaOptSeq;
                n = g_kfs.Count;
            }
            StringBuilder sb = new StringBuilder(n * 80 + 64);
            sb.Append("{\"ok\":true,\"optSeq\":").Append(opt);
            sb.Append(",\"requested\":").Append(reqOpt);
            sb.Append(",\"n\":").Append(d.Length / 16).Append(",\"d\":[");
            for (int i = 0; i < d.Length; i++)
            {
                if (i > 0) sb.Append(',');
                sb.Append(d[i].ToString("R", System.Globalization.CultureInfo.InvariantCulture));
            }
            sb.Append("]}");
            return sb.ToString();
        }

        static string ScanStopJson()
        {
            lock (g_trackLock)
            {
                try
                {
                    int nk = g_kfs.Count;
                    for (int c = Math.Max(11, nk - 4); c < nk; c++)
                    {
                        DetectLoops(c, nk - 12);
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine("ScanStop: " + ex.Message);
                }
                StringBuilder sb = new StringBuilder(128);
                sb.Append("{\"ok\":true,\"kf\":").Append(g_kfs.Count);
                sb.Append(",\"loops\":").Append(g_loops.Count);
                sb.Append(",\"optSeq\":").Append(g_optSeq);
                sb.Append(",\"state\":\"").Append(ScanStateName(g_trackState)).Append("\"}");
                Console.WriteLine("Scan stop: kf=" + g_kfs.Count + " loops=" + g_loops.Count);
                return sb.ToString();
            }
        }

        static void ElevLoop()
        {
            while (true)
            {
                int d = Interlocked.Exchange(ref g_elevPending, int.MinValue);
                if (d != int.MinValue) SetElevation(d);
                Thread.Sleep(100);
            }
        }

        static bool SetElevation(int deg)
        {
            try
            {
                if (g_sensor == null || !g_sensor.IsRunning)
                {
                    g_elevMsg = "sensore non attivo";
                    return false;
                }
                int min = g_sensor.MinElevationAngle;
                int max = g_sensor.MaxElevationAngle;
                if (deg < min) deg = min;
                if (deg > max) deg = max;
                g_sensor.ElevationAngle = deg;
                g_elevTarget = deg;
                g_elevMsg = "";
                Console.WriteLine("ElevationAngle -> " + deg + "°");
                return true;
            }
            catch (Exception ex)
            {
                g_elevMsg = ex.Message;
                Console.WriteLine("Errore elevation: " + ex.Message);
                return false;
            }
        }

        static void TouchFrame()
        {
            Interlocked.Exchange(ref g_lastFrameTicksUtc, DateTime.UtcNow.Ticks);
        }

        static long FrameAgeMs()
        {
            long ticks = Interlocked.Read(ref g_lastFrameTicksUtc);
            if (ticks <= 0) return -1;
            return (long)(DateTime.UtcNow - new DateTime(ticks, DateTimeKind.Utc)).TotalMilliseconds;
        }

        static void Main()
        {
            Console.OutputEncoding = Encoding.UTF8;
            Console.WriteLine("=== KinectBridge Robot 6 DOF ===");
            Console.WriteLine("Kinect for Windows SDK 1.8 -> http://127.0.0.1:" + Port);

            TcpListener listener = null;
            try
            {
                listener = new TcpListener(IPAddress.Loopback, Port);
                listener.Start();
            }
            catch (Exception ex)
            {
                Console.WriteLine("Porta " + Port + " gia in uso (bridge gia attivo?): " + ex.Message);
                return;
            }

            Thread sensorThread = new Thread(SensorLoop);
            sensorThread.IsBackground = true;
            sensorThread.Start();

            Thread elevWorker = new Thread(ElevLoop);
            elevWorker.IsBackground = true;
            elevWorker.Start();

            Thread trackWorker = new Thread(TrackLoop);
            trackWorker.IsBackground = true;
            trackWorker.Start();

            Console.WriteLine("In attesa di richieste HTTP...");
            while (true)
            {
                try
                {
                    TcpClient client = listener.AcceptTcpClient();
                    ThreadPool.QueueUserWorkItem(HandleClient, client);
                }
                catch (Exception ex)
                {
                    Console.WriteLine("Errore accept: " + ex.Message);
                }
            }
        }

        /* ================= SENSOR ================= */

        static void SensorLoop()
        {
            while (true)
            {
                try
                {
                    OpenSensor();
                }
                catch (Exception ex)
                {
                    g_error = ex.Message;
                    g_running = false;
                    Console.WriteLine("Errore apertura Kinect: " + ex.Message);
                }
                if (g_running)
                {
                    DateTime openedAt = DateTime.UtcNow;
                    while (g_sensor != null && g_sensor.IsRunning)
                    {
                        Thread.Sleep(500);
                        long age = FrameAgeMs();
                        if ((DateTime.UtcNow - openedAt).TotalSeconds > 8 &&
                            (age < 0 || age > 5000))
                        {
                            g_error = "Watchdog: frame fermi da " + (age < 0 ? "?" : age + " ms") + ", riavvio sensore";
                            g_restarts++;
                            Console.WriteLine(g_error + " (restart #" + g_restarts + ")");
                            break;
                        }
                    }
                    g_running = false;
                    if (string.IsNullOrEmpty(g_error))
                        g_error = "Sensore disconnesso o occupato da un'altra applicazione";
                    Console.WriteLine("Sensore non piu attivo, riprovo tra 2s...");
                }
                TryCloseSensor();
                Thread.Sleep(2000);
            }
        }

        static void OpenSensor()
        {
            g_error = "";
            if (KinectSensor.KinectSensors.Count == 0)
            {
                g_error = "Nessun Kinect rilevato. Controlla il cavo USB.";
                Console.WriteLine(g_error);
                return;
            }

            KinectSensor s = KinectSensor.KinectSensors[0];
            if (s.Status != KinectStatus.Connected)
            {
                g_error = "Kinect stato: " + s.Status;
                Console.WriteLine(g_error);
                return;
            }

            s.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
            s.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);

            g_dFx = s.DepthStream.NominalFocalLengthInPixels;
            g_dFy = s.DepthStream.NominalFocalLengthInPixels;
            g_cFx = s.ColorStream.NominalFocalLengthInPixels;
            g_cFy = s.ColorStream.NominalFocalLengthInPixels;
            g_minDepth = s.DepthStream.MinDepth;
            g_maxDepth = s.DepthStream.MaxDepth;
            g_tooNear = s.DepthStream.TooNearDepth;
            g_tooFar = s.DepthStream.TooFarDepth;
            g_unknown = s.DepthStream.UnknownDepth;
            g_deviceId = s.DeviceConnectionId;

            s.ColorFrameReady += OnColorFrame;
            s.DepthFrameReady += OnDepthFrame;

            s.Start();
            if (!s.IsRunning)
            {
                g_error = "Start() non riuscito";
                Console.WriteLine(g_error);
                return;
            }

            g_sensor = s;
            g_running = true;
            g_error = "";
            Console.WriteLine("Kinect avviata: " + g_deviceId);
            Console.WriteLine("Depth focal (px): " + g_dFx + "  Color focal (px): " + g_cFx);
            Console.WriteLine("Range affidabile: " + g_minDepth + "-" + g_maxDepth + " mm");
            StartAudio(s);
        }

        static void TryCloseSensor()
        {
            try
            {
                StopAudio();
                if (g_sensor != null)
                {
                    g_sensor.ColorFrameReady -= OnColorFrame;
                    g_sensor.DepthFrameReady -= OnDepthFrame;
                    if (g_sensor.IsRunning) g_sensor.Stop();
                    g_sensor.Dispose();
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine("Chiusura sensore: " + ex.Message);
            }
            g_sensor = null;
            Interlocked.Exchange(ref g_lastFrameTicksUtc, 0);
        }

        static void OnColorFrame(object sender, ColorImageFrameReadyEventArgs e)
        {
            try
            {
                using (ColorImageFrame f = e.OpenColorImageFrame())
                {
                    if (f == null) { g_nullFrames++; return; }

                    TouchFrame();
                    DateTime now = DateTime.UtcNow;
                    g_colorFrames++;
                    g_colorFpsCount++;
                    if ((now - g_colorFpsWin).TotalSeconds >= 1.0)
                    {
                        g_colorFps = g_colorFpsCount / (now - g_colorFpsWin).TotalSeconds;
                        g_colorFpsCount = 0;
                        g_colorFpsWin = now;
                    }

                    // codifica JPEG al massimo ~25 fps per non stressare la CPU
                    if ((now - g_lastJpeg).TotalMilliseconds < 40) return;
                    g_lastJpeg = now;

                    int len = f.BytesPerPixel * f.Width * f.Height;
                    byte[] raw = new byte[len];
                    f.CopyPixelDataTo(raw);

                    using (Bitmap bmp = new Bitmap(f.Width, f.Height, PixelFormat.Format32bppRgb))
                    {
                        BitmapData bd = bmp.LockBits(
                            new Rectangle(0, 0, f.Width, f.Height),
                            ImageLockMode.WriteOnly,
                            PixelFormat.Format32bppRgb);
                        System.Runtime.InteropServices.Marshal.Copy(raw, 0, bd.Scan0, len);
                        bmp.UnlockBits(bd);

                        using (MemoryStream ms = new MemoryStream())
                        {
                            ImageCodecInfo enc = GetJpegEncoder();
                            if (enc != null)
                            {
                                EncoderParameters ep = new EncoderParameters(1);
                                ep.Param[0] = new EncoderParameter(System.Drawing.Imaging.Encoder.Quality, 78L);
                                bmp.Save(ms, enc, ep);
                                byte[] jpeg = ms.ToArray();
                                lock (g_colorLock) { g_jpeg = jpeg; }
                            }
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                g_error = "Errore frame colore: " + ex.Message;
            }
        }

        static void OnDepthFrame(object sender, DepthImageFrameReadyEventArgs e)
        {
            try
            {
                using (DepthImageFrame f = e.OpenDepthImageFrame())
                {
                    if (f == null) { g_nullFrames++; return; }

                    TouchFrame();
                    DepthImagePixel[] px = f.GetRawPixelData();

                    int min = int.MaxValue;
                    int max = 0;
                    long sum = 0;
                    int valid = 0;
                    ushort[] buf = g_scratch;

                    for (int i = 0; i < px.Length; i++)
                    {
                        short d = px[i].Depth;
                        buf[i] = (ushort)(d > 0 ? d : 0);
                        if (d > 0)
                        {
                            valid++;
                            sum += d;
                            if (d < min) min = d;
                            if (d > max) max = d;
                        }
                    }

                    lock (g_depthLock)
                    {
                        ushort[] tmp = g_depth;
                        g_depth = buf;
                        g_scratch = tmp;
                    }
                    if (g_trackOn) g_trackPulse.Set();

                    g_statValid = valid;
                    g_statMin = valid > 0 ? min : 0;
                    g_statMax = max;
                    g_statAvg = valid > 0 ? (double)sum / valid : 0;

                    int cIdx = (H / 2) * W + (W / 2);
                    g_centerDepth = px[cIdx].Depth;

                    if (g_sensor != null && g_sensor.IsRunning && g_centerDepth > 0)
                    {
                        try
                        {
                            DepthImagePoint dip = new DepthImagePoint();
                            dip.X = W / 2;
                            dip.Y = H / 2;
                            dip.Depth = g_centerDepth;
                            SkeletonPoint sp = g_sensor.CoordinateMapper.MapDepthPointToSkeletonPoint(
                                DepthImageFormat.Resolution640x480Fps30, dip);
                            g_centerX = sp.X * 1000f;
                            g_centerY = sp.Y * 1000f;
                            g_centerZ = sp.Z * 1000f;
                        }
                        catch { g_centerX = g_centerY = g_centerZ = 0; }
                    }
                    else
                    {
                        g_centerX = g_centerY = g_centerZ = 0;
                    }

                    g_depthFrames++;
                    g_depthFpsCount++;
                    DateTime now = DateTime.UtcNow;
                    if ((now - g_depthFpsWin).TotalSeconds >= 1.0)
                    {
                        g_depthFps = g_depthFpsCount / (now - g_depthFpsWin).TotalSeconds;
                        g_depthFpsCount = 0;
                        g_depthFpsWin = now;
                    }
                }
            }
            catch (Exception ex)
            {
                g_error = "Errore frame profondita: " + ex.Message;
            }
        }

        static ImageCodecInfo GetJpegEncoder()
        {
            ImageCodecInfo[] encs = ImageCodecInfo.GetImageEncoders();
            for (int i = 0; i < encs.Length; i++)
            {
                if (encs[i].MimeType == "image/jpeg") return encs[i];
            }
            return null;
        }

        /* ================= HTTP ================= */

        static void HandleClient(Object state)
        {
            TcpClient client = (TcpClient)state;
            try
            {
                client.ReceiveTimeout = 5000;
                client.SendTimeout = 5000;
                NetworkStream ns = client.GetStream();

                byte[] head = ReadHeaders(ns);
                if (head == null) { client.Close(); return; }
                string header = Encoding.ASCII.GetString(head);
                string[] lines = header.Split(new string[] { "\r\n" }, StringSplitOptions.None);
                if (lines.Length == 0) { client.Close(); return; }

                string[] parts = lines[0].Split(' ');
                if (parts.Length < 2) { client.Close(); return; }
                string method = parts[0].ToUpperInvariant();
                string url = parts[1];

                if (method == "OPTIONS")
                {
                    Send(client, 204, "No Content", "text/plain", new byte[0]);
                    return;
                }
                if (method != "GET")
                {
                    Send(client, 405, "Method Not Allowed", "text/plain", Encoding.UTF8.GetBytes("Solo GET"));
                    return;
                }

                int q = url.IndexOf('?');
                string path = (q >= 0 ? url.Substring(0, q) : url).Trim('/');
                string query = q >= 0 ? url.Substring(q + 1) : "";

                if (path == "" || path == "status")
                {
                    SendJson(client, BuildStatusJson());
                }
                else if (path == "color.jpg")
                {
                    byte[] jpeg;
                    lock (g_colorLock) { jpeg = g_jpeg; }
                    if (jpeg == null)
                    {
                        SendJson(client, "{\"ok\":false,\"error\":\"nessun frame colore\"}", 503);
                    }
                    else
                    {
                        Send(client, 200, "OK", "image/jpeg", jpeg);
                    }
                }
                else if (path == "depth.bin")
                {
                    if (QueryInt(query, "pose", 0) == 1)
                    {
                        Send(client, 200, "OK", "application/octet-stream", BuildDepthWithPose());
                    }
                    else
                    {
                        byte[] bytes;
                        lock (g_depthLock)
                        {
                            bytes = new byte[g_depth.Length * 2];
                            Buffer.BlockCopy(g_depth, 0, bytes, 0, bytes.Length);
                        }
                        Send(client, 200, "OK", "application/octet-stream", bytes);
                    }
                }
                else if (path == "scan/track")
                {
                    int on = QueryInt(query, "on", -1);
                    if (on < 0) SendJson(client, "{\"ok\":false,\"error\":\"parametro on mancante\"}");
                    else
                    {
                        ScanTrackSet(on == 1);
                        SendJson(client, "{\"ok\":true,\"track\":" + (on == 1 ? "true" : "false") + "}");
                    }
                }
                else if (path == "scan/reset")
                {
                    ScanReset();
                    SendJson(client, "{\"ok\":true}");
                }
                else if (path == "scan/stop")
                {
                    SendJson(client, ScanStopJson());
                }
                else if (path == "scan/deltas")
                {
                    int opt = QueryInt(query, "opt", -1);
                    SendJson(client, BuildDeltasJson(opt));
                }
                else if (path == "map")
                {
                    SendJson(client, BuildMapJson(query));
                }
                else if (path == "elev" || path == "tilt")
                {
                    int deg = QueryInt(query, "deg", int.MinValue);
                    bool ok;
                    string err = "";
                    int target = g_elevTarget;
                    if (deg == int.MinValue)
                    {
                        err = "parametro deg mancante";
                        ok = false;
                    }
                    else if (g_sensor == null || !g_sensor.IsRunning)
                    {
                        err = "sensore non attivo";
                        ok = false;
                    }
                    else
                    {
                        int min = -27, max = 27;
                        try { min = g_sensor.MinElevationAngle; max = g_sensor.MaxElevationAngle; }
                        catch { }
                        if (deg < min) deg = min;
                        if (deg > max) deg = max;
                        g_elevPending = deg;
                        target = deg;
                        g_elevMsg = "";
                        ok = true;
                    }
                    StringBuilder eb = new StringBuilder(128);
                    eb.Append("{\"ok\":").Append(ok ? "true" : "false");
                    eb.Append(",\"target\":").Append(target);
                    if (!ok) eb.Append(",\"error\":\"").Append(Escape(err)).Append("\"");
                    eb.Append("}");
                    SendJson(client, eb.ToString());
                }
                else if (path == "audio/status" || path == "audio/beam")
                {
                    SendJson(client, "{\"ok\":true," + BuildAudioFields() + "}");
                }
                else if (path == "audio/pcm" || path == "audio/wav")
                {
                    int ms = QueryInt(query, "ms", path == "audio/wav" ? 3000 : 1000);
                    if (ms < 50) ms = 50;
                    if (ms > 15000) ms = 15000;
                    byte[] pcm = ReadRing(ms * AudioBytesPerMs);
                    if (path == "audio/wav")
                    {
                        Send(client, 200, "OK", "audio/wav", WrapWav(pcm));
                    }
                    else
                    {
                        Send(client, 200, "OK", "application/octet-stream", pcm);
                    }
                }
                else
                {
                    Send(client, 404, "Not Found", "text/plain", Encoding.UTF8.GetBytes("Endpoint sconosciuto"));
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine("HTTP: " + ex.Message);
            }
            finally
            {
                try { client.Close(); } catch { }
            }
        }

        static byte[] ReadHeaders(NetworkStream ns)
        {
            List<byte> buf = new List<byte>(4096);
            byte[] tmp = new byte[1024];
            while (buf.Count < 16384)
            {
                int r = ns.Read(tmp, 0, tmp.Length);
                if (r <= 0) return null;
                for (int i = 0; i < r; i++) buf.Add(tmp[i]);
                if (buf.Count >= 4)
                {
                    int n = buf.Count;
                    if (buf[n - 1] == '\n' && buf[n - 2] == '\r' && buf[n - 3] == '\n' && buf[n - 4] == '\r')
                        return buf.ToArray();
                }
            }
            return buf.ToArray();
        }

        static void SendJson(TcpClient client, string json)
        {
            Send(client, 200, "OK", "application/json; charset=utf-8", Encoding.UTF8.GetBytes(json));
        }

        static void SendJson(TcpClient client, string json, int code)
        {
            string status = code == 503 ? "Service Unavailable" : "OK";
            Send(client, code, status, "application/json; charset=utf-8", Encoding.UTF8.GetBytes(json));
        }

        static void Send(TcpClient client, int code, string status, string contentType, byte[] body)
        {
            StringBuilder sb = new StringBuilder(256);
            sb.Append("HTTP/1.1 ").Append(code).Append(' ').Append(status).Append("\r\n");
            sb.Append("Content-Type: ").Append(contentType).Append("\r\n");
            sb.Append("Content-Length: ").Append(body.Length).Append("\r\n");
            sb.Append("Access-Control-Allow-Origin: *\r\n");
            sb.Append("Access-Control-Allow-Methods: GET, OPTIONS\r\n");
            sb.Append("Access-Control-Allow-Headers: Content-Type\r\n");
            sb.Append("Cache-Control: no-store, no-cache, must-revalidate\r\n");
            sb.Append("Pragma: no-cache\r\n");
            sb.Append("Connection: close\r\n\r\n");

            byte[] head = Encoding.ASCII.GetBytes(sb.ToString());
            NetworkStream ns = client.GetStream();
            ns.Write(head, 0, head.Length);
            if (body.Length > 0) ns.Write(body, 0, body.Length);
            ns.Flush();
        }

        static string BuildStatusJson()
        {
            StringBuilder sb = new StringBuilder(1024);
            sb.Append("{");
            sb.Append("\"ok\":true,");
            sb.Append("\"running\":").Append(g_running ? "true" : "false").Append(',');
            sb.Append("\"error\":\"").Append(Escape(g_error)).Append("\",");
            sb.Append("\"device\":\"").Append(Escape(g_deviceId)).Append("\",");
            sb.Append("\"sdk\":\"Kinect for Windows SDK 1.8\",");
            sb.Append("\"colorFrames\":").Append(g_colorFrames).Append(',');
            sb.Append("\"depthFrames\":").Append(g_depthFrames).Append(',');
            sb.Append("\"colorFps\":").Append(D1(g_colorFps)).Append(',');
            sb.Append("\"depthFps\":").Append(D1(g_depthFps)).Append(',');
            sb.Append("\"color\":{\"w\":").Append(W).Append(",\"h\":").Append(H)
              .Append(",\"fx\":").Append(F(g_cFx)).Append(",\"fy\":").Append(F(g_cFy))
              .Append(",\"cx\":").Append(W / 2).Append(",\"cy\":").Append(H / 2).Append("},");
            sb.Append("\"depth\":{\"w\":").Append(W).Append(",\"h\":").Append(H)
              .Append(",\"fx\":").Append(F(g_dFx)).Append(",\"fy\":").Append(F(g_dFy))
              .Append(",\"cx\":").Append(W / 2).Append(",\"cy\":").Append(H / 2)
              .Append(",\"minDepth\":").Append(g_minDepth)
              .Append(",\"maxDepth\":").Append(g_maxDepth)
              .Append(",\"tooNear\":").Append(g_tooNear)
              .Append(",\"tooFar\":").Append(g_tooFar)
              .Append(",\"unknown\":").Append(g_unknown).Append("},");
            sb.Append("\"stats\":{\"min\":").Append(g_statMin)
              .Append(",\"max\":").Append(g_statMax)
              .Append(",\"avg\":").Append(g_statAvg.ToString("0", System.Globalization.CultureInfo.InvariantCulture))
              .Append(",\"valid\":").Append(g_statValid)
              .Append(",\"total\":").Append(N)
              .Append(",\"centerDepth\":").Append(g_centerDepth).Append("},");
            sb.Append("\"center\":{\"x\":").Append(F(g_centerX))
              .Append(",\"y\":").Append(F(g_centerY))
              .Append(",\"z\":").Append(F(g_centerZ)).Append("}");
            sb.Append(",\"lastFrameAgeMs\":").Append(FrameAgeMs());
            sb.Append(",\"nullFrames\":").Append(Interlocked.Read(ref g_nullFrames));
            sb.Append(",\"restarts\":").Append(g_restarts);
            int elevNow = g_elevTarget;
            int elevMin = -27, elevMax = 27;
            try
            {
                if (g_sensor != null)
                {
                    elevMin = g_sensor.MinElevationAngle;
                    elevMax = g_sensor.MaxElevationAngle;
                    if (g_sensor.IsRunning) elevNow = g_sensor.ElevationAngle;
                }
            }
            catch { }
            sb.Append(",\"elev\":").Append(elevNow);
            sb.Append(",\"elevTarget\":").Append(g_elevTarget);
            sb.Append(",\"elevMin\":").Append(elevMin);
            sb.Append(",\"elevMax\":").Append(elevMax);
            if (g_elevMsg != "") sb.Append(",\"elevError\":\"").Append(Escape(g_elevMsg)).Append("\"");
            sb.Append(",\"audio\":{").Append(BuildAudioFields()).Append("}");
            sb.Append(",\"scan\":{").Append(BuildScanFields()).Append("}");
            sb.Append("}");
            return sb.ToString();
        }

        static string BuildMapJson(string query)
        {
            int x = QueryInt(query, "x", -1);
            int y = QueryInt(query, "y", -1);
            if (x < 0 || y < 0 || x >= W || y >= H)
                return "{\"ok\":false,\"error\":\"x/y fuori range 0-639 / 0-479\"}";

            ushort d;
            lock (g_depthLock) { d = g_depth[y * W + x]; }
            if (d <= 0)
                return "{\"ok\":false,\"error\":\"pixel senza dati di profondita\",\"x\":" + x + ",\"y\":" + y + ",\"depth\":0}";

            float mx = 0, my = 0, mz = 0;
            bool mapped = false;
            if (g_sensor != null && g_sensor.IsRunning)
            {
                try
                {
                    DepthImagePoint dip = new DepthImagePoint();
                    dip.X = x;
                    dip.Y = y;
                    dip.Depth = d;
                    SkeletonPoint sp = g_sensor.CoordinateMapper.MapDepthPointToSkeletonPoint(
                        DepthImageFormat.Resolution640x480Fps30, dip);
                    mx = sp.X * 1000f;
                    my = sp.Y * 1000f;
                    mz = sp.Z * 1000f;
                    mapped = true;
                }
                catch { mapped = false; }
            }

            if (!mapped)
            {
                // proiezione nominale come fallback
                mx = ((x - W / 2) * d) / g_dFx;
                my = ((y - H / 2) * d) / g_dFy;
                mz = d;
            }

            StringBuilder sb = new StringBuilder(256);
            sb.Append("{\"ok\":true,\"x\":").Append(x)
              .Append(",\"y\":").Append(y)
              .Append(",\"depth\":").Append(d)
              .Append(",\"mx\":").Append(F(mx))
              .Append(",\"my\":").Append(F(my))
              .Append(",\"mz\":").Append(F(mz))
              .Append(",\"mapped\":").Append(mapped ? "true" : "false")
              .Append("}");
            return sb.ToString();
        }

        static int QueryInt(string query, string key, int def)
        {
            string[] pairs = query.Split('&');
            for (int i = 0; i < pairs.Length; i++)
            {
                string[] kv = pairs[i].Split('=');
                if (kv.Length == 2 && kv[0] == key)
                {
                    int v;
                    if (int.TryParse(kv[1], out v)) return v;
                }
            }
            return def;
        }

        static string F(float v)
        {
            if (float.IsNaN(v) || float.IsInfinity(v)) return "0";
            return v.ToString("0.##", System.Globalization.CultureInfo.InvariantCulture);
        }

        static string D1(double v)
        {
            if (double.IsNaN(v) || double.IsInfinity(v)) return "0";
            return v.ToString("0.0", System.Globalization.CultureInfo.InvariantCulture);
        }

        static string Escape(string s)
        {
            if (string.IsNullOrEmpty(s)) return "";
            return s.Replace("\\", "\\\\").Replace("\"", "\\\"")
                    .Replace("\r", " ").Replace("\n", " ").Replace("\t", " ");
        }
    }
}
