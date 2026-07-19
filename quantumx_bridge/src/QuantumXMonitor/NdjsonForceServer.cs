using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using Newtonsoft.Json;

namespace QuantumXMonitor
{
    internal sealed class NdjsonForceServer : IDisposable
    {
        private readonly TcpListener _listener;
        private readonly List<TcpClient> _clients = new List<TcpClient>();
        private readonly object _clientsLock = new object();
        private Thread _acceptThread;
        private volatile bool _running;
        private long _sequence;

        public NdjsonForceServer(int port)
        {
            _listener = new TcpListener(IPAddress.Loopback, port);
        }

        public void Start()
        {
            if (_running)
            {
                return;
            }

            _listener.Start();
            _running = true;
            _acceptThread = new Thread(AcceptClients)
            {
                IsBackground = true,
                Name = "quantumx-ndjson-accept"
            };
            _acceptThread.Start();
        }

        public void Publish(FilteredSample sample)
        {
            long sequence = Interlocked.Increment(ref _sequence);
            long timestampUtcNs = (DateTime.UtcNow.Ticks - 621355968000000000L) * 100L;
            string line = JsonConvert.SerializeObject(new
            {
                schema_version = 1,
                sequence,
                timestamp_utc_ns = timestampUtcNs,
                force_1_n = sample.Force1N,
                force_2_n = sample.Force2N,
                force_total_n = sample.ForceTotalN,
                status = "ok",
                channel_1_status = "ok",
                channel_2_status = "ok"
            }) + "\n";
            byte[] payload = new UTF8Encoding(false).GetBytes(line);

            lock (_clientsLock)
            {
                for (int index = _clients.Count - 1; index >= 0; index--)
                {
                    TcpClient client = _clients[index];
                    try
                    {
                        NetworkStream stream = client.GetStream();
                        stream.Write(payload, 0, payload.Length);
                    }
                    catch (Exception exception) when (
                        exception is System.IO.IOException ||
                        exception is ObjectDisposedException ||
                        exception is SocketException)
                    {
                        client.Close();
                        _clients.RemoveAt(index);
                    }
                }
            }
        }

        private void AcceptClients()
        {
            while (_running)
            {
                try
                {
                    TcpClient client = _listener.AcceptTcpClient();
                    client.NoDelay = true;
                    lock (_clientsLock)
                    {
                        if (_running)
                        {
                            _clients.Add(client);
                        }
                        else
                        {
                            client.Close();
                        }
                    }
                }
                catch (SocketException)
                {
                    if (_running)
                    {
                        throw;
                    }
                }
                catch (ObjectDisposedException)
                {
                    return;
                }
            }
        }

        public void Dispose()
        {
            _running = false;
            _listener.Stop();
            if (_acceptThread != null && _acceptThread != Thread.CurrentThread)
            {
                _acceptThread.Join(1000);
            }

            lock (_clientsLock)
            {
                foreach (TcpClient client in _clients)
                {
                    client.Close();
                }
                _clients.Clear();
            }
        }
    }
}
