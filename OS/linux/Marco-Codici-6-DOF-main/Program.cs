// NOTA (versione Linux): questo launcher usa WinForms/NotifyIcon e quindi
// funziona solo su Windows. Su Linux avviare il progetto con:
//   scripts/AVVIA_SIMULATORE.sh   oppure   node server.js
using System;
using System.IO;
using System.Net;
using System.Text;
using System.Diagnostics;
using System.Windows.Forms;
using System.Drawing;
using System.Collections.Generic;

namespace Robot6DOFLauncher
{
    static class Program
    {
        private static HttpListener listener;
        private static string workDir;
        private static int port = 8765;
        
        // Simulazione ESP32 virtuale (PC mode)
        private static List<string> virtualEsp32Log = new List<string>();
        private static bool virtualEsp32Connected = true;

        [STAThread]
        static void Main()
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);

            // L'eseguibile vive in dist/, mentre HTML, modelli e configurazioni
            // vivono nella cartella principale del progetto.
            string exeDir = AppDomain.CurrentDomain.BaseDirectory;
            string projectDir = Path.GetFullPath(Path.Combine(exeDir, ".."));
            workDir = File.Exists(Path.Combine(projectDir, "ik_simulator_v30.html"))
                ? projectDir
                : exeDir;

            // Start internal HTTP Server
            StartServer();

            // Open in default browser or webview window
            string url = "http://localhost:" + port + "/ik_simulator_v30.html";
            try
            {
                Process.Start(new ProcessStartInfo
                {
                    FileName = url,
                    UseShellExecute = true
                });
            }
            catch (Exception ex)
            {
                MessageBox.Show("Errore apertura browser: " + ex.Message);
            }

            // Keep background server running in system tray or lightweight form
            Application.Run(new AppContext());
        }

        static void AtomicWriteJson(string filePath, string body)
        {
            string tempPath = filePath + ".tmp";
            File.WriteAllText(tempPath, body, Encoding.UTF8);
            if (File.Exists(filePath)) File.Delete(filePath);
            File.Move(tempPath, filePath);
        }

        static void StartServer()
        {
            try
            {
                listener = new HttpListener();
                listener.Prefixes.Add("http://localhost:" + port + "/");
                try
                {
                    listener.Start();
                }
                catch
                {
                    // Fallback to wildcard or specific loopback
                    listener.Close();
                    listener = new HttpListener();
                    listener.Prefixes.Add("http://127.0.0.1:" + port + "/");
                    listener.Start();
                }
                listener.BeginGetContext(OnContext, null);
                File.WriteAllText(Path.Combine(workDir, "config", "server_status.txt"), 
                    "RUNNING on port " + port + " at " + DateTime.Now.ToString() + "\nMODE: PC_SIMULATION");
            }
catch (Exception ex)
                {
                    // Log error to file and console
                    string errorMsg = "ERROR: " + ex.ToString() + " at " + DateTime.Now.ToString();
                    try { File.WriteAllText(Path.Combine(workDir, "config", "server_error.txt"), errorMsg + "\n", Encoding.UTF8); } catch { }
                    Console.WriteLine(errorMsg);
                }
        }

        static void OnContext(IAsyncResult ar)
        {
            if (listener == null || !listener.IsListening) return;
            try
            {
                HttpListenerContext ctx = listener.EndGetContext(ar);
                listener.BeginGetContext(OnContext, null);

                HttpListenerRequest req = ctx.Request;
                HttpListenerResponse resp = ctx.Response;

                resp.Headers.Add("Access-Control-Allow-Origin", "*");
                resp.Headers.Add("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
                resp.Headers.Add("Access-Control-Allow-Headers", "Content-Type");
                resp.Headers.Add("Cache-Control", "no-cache, no-store, must-revalidate, max-age=0");
                resp.Headers.Add("Pragma", "no-cache");
                resp.Headers.Add("Expires", "0");

                if (req.HttpMethod == "OPTIONS")
                {
                    resp.StatusCode = 200;
                    resp.Close();
                    return;
                }

                string rawUrl = req.RawUrl.TrimStart('/');
                if (string.IsNullOrEmpty(rawUrl) || rawUrl == "") rawUrl = "ik_simulator_v30.html";

                // === API MODALITA' OPERATIVA ===
                if (rawUrl.StartsWith("api/mode"))
                {
                    byte[] data = Encoding.UTF8.GetBytes("{\"mode\":\"simulation\",\"platform\":\"windows\"}");
                    resp.ContentType = "application/json";
                    resp.OutputStream.Write(data, 0, data.Length);
                    resp.Close();
                    return;
                }

                // === API ESP32 VIRTUALE (SIMULAZIONE PC) ===
                if (rawUrl.StartsWith("api/esp32/status") && req.HttpMethod == "GET")
                {
                    byte[] data = Encoding.UTF8.GetBytes(
                        "{\"connected\":" + (virtualEsp32Connected ? "true" : "false") + 
                        ",\"port\":\"VIRTUAL_SIMULATION\",\"mode\":\"simulation\"}");
                    resp.ContentType = "application/json";
                    resp.OutputStream.Write(data, 0, data.Length);
                    resp.Close();
                    return;
                }

                if (rawUrl.StartsWith("api/esp32/send") && req.HttpMethod == "POST")
                {
                    using (var reader = new StreamReader(req.InputStream, req.ContentEncoding))
                    {
                        string body = reader.ReadToEnd();
                        // Parse JSON command
                        if (body.Contains("\"command\""))
                        {
                            int startIdx = body.IndexOf("\"command\":\"") + 11;
                            int endIdx = body.IndexOf("\"", startIdx);
                            if (startIdx > 10 && endIdx > startIdx)
                            {
                                string command = body.Substring(startIdx, endIdx - startIdx);
                                // Log virtuale
                                lock (virtualEsp32Log)
                                {
                                    virtualEsp32Log.Add("[SIM] " + DateTime.Now.ToString("HH:mm:ss") + " >> " + command);
                                    if (virtualEsp32Log.Count > 100) virtualEsp32Log.RemoveAt(0);
                                }
                                Console.WriteLine("[VIRTUAL ESP32] Comando simulato: " + command);
                            }
                        }
                    }
                    byte[] ok = Encoding.UTF8.GetBytes("{\"success\":true,\"simulated\":true}");
                    resp.ContentType = "application/json";
                    resp.OutputStream.Write(ok, 0, ok.Length);
                    resp.Close();
                    return;
                }

                if (rawUrl.StartsWith("api/esp32/logs") && req.HttpMethod == "GET")
                {
                    string logsJson = "[";
                    lock (virtualEsp32Log)
                    {
                        for (int i = 0; i < virtualEsp32Log.Count; i++)
                        {
                            logsJson += "\"" + virtualEsp32Log[i].Replace("\"", "\\\"") + "\"";
                            if (i < virtualEsp32Log.Count - 1) logsJson += ",";
                        }
                    }
                    logsJson += "]";
                    byte[] data = Encoding.UTF8.GetBytes(logsJson);
                    resp.ContentType = "application/json";
                    resp.OutputStream.Write(data, 0, data.Length);
                    resp.Close();
                    return;
                }

                if (rawUrl.StartsWith("api/esp32/reconnect") && req.HttpMethod == "POST")
                {
                    virtualEsp32Connected = true;
                    byte[] ok = Encoding.UTF8.GetBytes("{\"success\":true,\"connected\":true,\"simulated\":true}");
                    resp.ContentType = "application/json";
                    resp.OutputStream.Write(ok, 0, ok.Length);
                    resp.Close();
                    return;
                }

                // API per salvataggio e caricamento automatico
                if (rawUrl.StartsWith("api/save_stl_config") && req.HttpMethod == "POST")
                {
                    using (var reader = new StreamReader(req.InputStream, req.ContentEncoding))
                    {
                        string body = reader.ReadToEnd();
                        AtomicWriteJson(Path.Combine(workDir, "config", "robot6dof_stl_config.json"), body);
                    }
                    byte[] ok = Encoding.UTF8.GetBytes("{\"status\":\"ok\"}");
                    resp.ContentType = "application/json";
                    resp.OutputStream.Write(ok, 0, ok.Length);
                    resp.Close();
                    return;
                }

                if (rawUrl.StartsWith("api/save_poses_config") && req.HttpMethod == "POST")
                {
                    using (var reader = new StreamReader(req.InputStream, req.ContentEncoding))
                    {
                        string body = reader.ReadToEnd();
                        AtomicWriteJson(Path.Combine(workDir, "config", "robot6dof_saved_poses.json"), body);
                    }
                    byte[] ok = Encoding.UTF8.GetBytes("{\"status\":\"ok\"}");
                    resp.ContentType = "application/json";
                    resp.OutputStream.Write(ok, 0, ok.Length);
                    resp.Close();
                    return;
                }

                if (rawUrl.StartsWith("api/save_anim_config") && req.HttpMethod == "POST")
                {
                    using (var reader = new StreamReader(req.InputStream, req.ContentEncoding))
                    {
                        string body = reader.ReadToEnd();
                        AtomicWriteJson(Path.Combine(workDir, "config", "robot6dof_animations.json"), body);
                    }
                    byte[] ok = Encoding.UTF8.GetBytes("{\"status\":\"ok\"}");
                    resp.ContentType = "application/json";
                    resp.OutputStream.Write(ok, 0, ok.Length);
                    resp.Close();
                    return;
                }

                if (rawUrl.StartsWith("api/list_stls"))
                {
                    var files = Directory.GetFiles(Path.Combine(workDir, "models"), "*.stl");
                    var sb = new StringBuilder("[");
                    for (int i = 0; i < files.Length; i++)
                    {
                        string name = Path.GetFileName(files[i]);
                        sb.Append("\"").Append(name).Append("\"");
                        if (i < files.Length - 1) sb.Append(",");
                    }
                    sb.Append("]");
                    byte[] data = Encoding.UTF8.GetBytes(sb.ToString());
                    resp.ContentType = "application/json";
                    resp.OutputStream.Write(data, 0, data.Length);
                    resp.Close();
                    return;
                }

                // Servire file statici (HTML, STL, JSON, JS)
                string cleanPath = rawUrl.Split('?')[0];
                string filePath = Path.Combine(workDir, cleanPath);
                if (File.Exists(filePath))
                {
                    byte[] bytes = File.ReadAllBytes(filePath);
                    string ext = Path.GetExtension(filePath).ToLower();
                    if (ext == ".html") resp.ContentType = "text/html; charset=utf-8";
                    else if (ext == ".json") resp.ContentType = "application/json; charset=utf-8";
                    else if (ext == ".stl") resp.ContentType = "application/octet-stream";
                    else if (ext == ".js") resp.ContentType = "application/javascript";
                    else if (ext == ".css") resp.ContentType = "text/css";

                    resp.ContentLength64 = bytes.Length;
                    resp.OutputStream.Write(bytes, 0, bytes.Length);
                    resp.Close();
                }
                else
                {
                    resp.StatusCode = 404;
                    byte[] nf = Encoding.UTF8.GetBytes("File non trovato");
                    resp.OutputStream.Write(nf, 0, nf.Length);
                    resp.Close();
                }
            }
            catch (Exception ex)
            {
                // Log error to console but continue server operation
                Console.WriteLine("HTTP request error: " + ex.Message);
            }
        }

        class AppContext : ApplicationContext
        {
            private NotifyIcon trayIcon;

            public AppContext()
            {
                trayIcon = new NotifyIcon()
                {
                    Text = "Digital Twin 6 DOF - Server Attivo (PC Simulazione)",
                    Icon = SystemIcons.Application,
                    Visible = true
                };

                var contextMenu = new ContextMenuStrip();
                contextMenu.Items.Add("Apri Simulatore (v30)", null, (s, e) => {
                    Process.Start(new ProcessStartInfo("http://localhost:" + port + "/ik_simulator_v30.html") { UseShellExecute = true });
                });
                contextMenu.Items.Add("Apri Cartella Marco Codici 6 DOF", null, (s, e) => {
                    // explorer.exe esiste solo su Windows: su Linux usare il file manager
                    if (Environment.OSVersion.Platform == PlatformID.Win32NT)
                    {
                        Process.Start(new ProcessStartInfo("explorer.exe", workDir));
                    }
                });
                contextMenu.Items.Add(new ToolStripSeparator());
                contextMenu.Items.Add("Esci", null, (s, e) => {
                    trayIcon.Visible = false;
                    Application.Exit();
                });

                trayIcon.ContextMenuStrip = contextMenu;
            }
        }
    }
}
