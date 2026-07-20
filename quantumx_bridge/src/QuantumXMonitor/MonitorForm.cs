using System;
using System.Drawing;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace QuantumXMonitor
{
    internal sealed class MonitorForm : Form
    {
        private const string DeviceIp = "192.168.10.20";
        private readonly Label _force1Value;
        private readonly Label _force2Value;
        private readonly Label _totalValue;
        private readonly Label _statusLabel;
        private readonly Label _windowLabel;
        private readonly Button _zeroBothButton;
        private readonly Button _reconnectButton;
        private readonly NdjsonForceServer _forceServer;
        private CancellationTokenSource _cancellation;
        private Task _readerTask;
        private QuantumXReader _reader;

        public MonitorForm(bool serverOnly = false)
        {
            Text = "MX440B Force Monitor";
            StartPosition = FormStartPosition.CenterScreen;
            MinimumSize = new Size(760, 360);
            Size = new Size(980, 430);
            BackColor = Color.FromArgb(245, 247, 250);
            Font = new Font("Segoe UI", 10F);

            var heading = new Label
            {
                Text = "Force measurement - average over 100 samples",
                Dock = DockStyle.Top,
                Height = 58,
                TextAlign = ContentAlignment.MiddleCenter,
                Font = new Font("Segoe UI", 18F, FontStyle.Bold),
                ForeColor = Color.FromArgb(32, 43, 55)
            };

            var valuesTable = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 3,
                RowCount = 1,
                Padding = new Padding(18, 8, 18, 12)
            };
            valuesTable.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.333F));
            valuesTable.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.333F));
            valuesTable.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.334F));

            _force1Value = CreateValueCard(valuesTable, 0, "Sensor 1");
            _force2Value = CreateValueCard(valuesTable, 1, "Sensor 2");
            _totalValue = CreateValueCard(valuesTable, 2, "Total");

            var footer = new Panel
            {
                Dock = DockStyle.Bottom,
                Height = 72,
                BackColor = Color.White,
                Padding = new Padding(18, 10, 18, 10)
            };

            _statusLabel = new Label
            {
                Text = "Not connected",
                AutoSize = false,
                Location = new Point(18, 10),
                Size = new Size(610, 24),
                ForeColor = Color.DarkOrange,
                Font = new Font("Segoe UI", 10F, FontStyle.Bold)
            };
            _windowLabel = new Label
            {
                Text = "Averaging window: 0 / 100",
                AutoSize = false,
                Location = new Point(18, 38),
                Size = new Size(610, 22),
                ForeColor = Color.DimGray
            };
            _reconnectButton = new Button
            {
                Text = "Reconnect",
                Anchor = AnchorStyles.Top | AnchorStyles.Right,
                Size = new Size(150, 38),
                Location = new Point(ClientSize.Width - 168, 17),
                Enabled = false
            };
            _reconnectButton.Click += (sender, args) => StartReader();
            _zeroBothButton = new Button
            {
                Text = "Zero both sensors",
                Anchor = AnchorStyles.Top | AnchorStyles.Right,
                Size = new Size(150, 38),
                Location = new Point(ClientSize.Width - 326, 17),
                Enabled = false
            };
            _zeroBothButton.Click += (sender, args) => ZeroBothSensors();
            footer.Resize += (sender, args) =>
            {
                _reconnectButton.Left = footer.ClientSize.Width - _reconnectButton.Width - 18;
                _zeroBothButton.Left = _reconnectButton.Left - _zeroBothButton.Width - 8;
                int labelWidth = Math.Max(200, _zeroBothButton.Left - 36);
                _statusLabel.Width = labelWidth;
                _windowLabel.Width = labelWidth;
            };
            footer.Controls.Add(_statusLabel);
            footer.Controls.Add(_windowLabel);
            footer.Controls.Add(_zeroBothButton);
            footer.Controls.Add(_reconnectButton);

            Controls.Add(valuesTable);
            Controls.Add(footer);
            Controls.Add(heading);

            if (serverOnly)
            {
                ShowInTaskbar = false;
                WindowState = FormWindowState.Minimized;
                Opacity = 0;
            }

            _forceServer = new NdjsonForceServer(5500);
            _forceServer.Start();

            Shown += (sender, args) => StartReader();
            FormClosing += OnFormClosing;
        }

        private static Label CreateValueCard(TableLayoutPanel table, int column, string title)
        {
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                Margin = new Padding(8),
                BackColor = Color.White,
                BorderStyle = BorderStyle.FixedSingle
            };
            var titleLabel = new Label
            {
                Text = title,
                Dock = DockStyle.Top,
                Height = 52,
                TextAlign = ContentAlignment.BottomCenter,
                Font = new Font("Segoe UI", 14F, FontStyle.Bold),
                ForeColor = Color.FromArgb(65, 78, 91)
            };
            var valueLabel = new Label
            {
                Text = "— N",
                Dock = DockStyle.Fill,
                TextAlign = ContentAlignment.MiddleCenter,
                Font = new Font("Segoe UI", 28F, FontStyle.Bold),
                ForeColor = column == 2 ? Color.FromArgb(0, 105, 92) : Color.FromArgb(26, 83, 126)
            };
            panel.Controls.Add(valueLabel);
            panel.Controls.Add(titleLabel);
            table.Controls.Add(panel, column, 0);
            return valueLabel;
        }

        private void StartReader()
        {
            if (_readerTask != null && !_readerTask.IsCompleted)
            {
                return;
            }

            _reconnectButton.Enabled = false;
            _zeroBothButton.Enabled = false;
            _force1Value.Text = "— N";
            _force2Value.Text = "— N";
            _totalValue.Text = "— N";
            _windowLabel.Text = "Averaging window: 0 / 100";
            SetStatus("Connecting ...", Color.DarkOrange);

            _cancellation = new CancellationTokenSource();
            var reader = new QuantumXReader(DeviceIp);
            _reader = reader;
            long lastUiUpdateTicks = 0;
            reader.StatusChanged += status => PostToUi(() =>
            {
                SetStatus(status, StatusColor(status));
                UpdateZeroButton(status);
            });
            reader.SampleReceived += sample =>
            {
                _forceServer.Publish(sample);
                long nowTicks = DateTime.UtcNow.Ticks;
                if (nowTicks - lastUiUpdateTicks >= TimeSpan.TicksPerSecond / 20)
                {
                    lastUiUpdateTicks = nowTicks;
                    PostToUi(() => DisplaySample(sample));
                }
            };

            _readerTask = Task.Run(() => reader.Run(_cancellation.Token), _cancellation.Token);
            _readerTask.ContinueWith(task =>
            {
                if (task.IsCanceled)
                {
                    return;
                }

                string message = task.Exception == null
                    ? "Measurement stopped"
                    : "Error: " + task.Exception.GetBaseException().Message;
                if (task.Exception != null)
                {
                    WriteLog(task.Exception.GetBaseException().ToString());
                }
                PostToUi(() =>
                {
                    _reader = null;
                    SetStatus(message, Color.Firebrick);
                    _zeroBothButton.Enabled = false;
                    _reconnectButton.Enabled = true;
                });
            }, TaskScheduler.Default);
        }

        private void ZeroBothSensors()
        {
            QuantumXReader reader = _reader;
            if (reader == null || !reader.RequestZeroBoth())
            {
                return;
            }

            _zeroBothButton.Enabled = false;
            SetStatus("Zero requested ...", Color.DarkOrange);
        }

        private void UpdateZeroButton(string status)
        {
            if (status.StartsWith("Connected", StringComparison.OrdinalIgnoreCase) ||
                status.StartsWith("Fallback mode", StringComparison.OrdinalIgnoreCase) ||
                status.StartsWith("Zero complete", StringComparison.OrdinalIgnoreCase) ||
                status.StartsWith("Zero failed", StringComparison.OrdinalIgnoreCase))
            {
                _zeroBothButton.Enabled = true;
            }
            else if (!status.StartsWith("Overrange", StringComparison.OrdinalIgnoreCase))
            {
                _zeroBothButton.Enabled = false;
            }
        }

        private void DisplaySample(FilteredSample sample)
        {
            _force1Value.Text = sample.FastForce1N.ToString("0.000") + " N";
            _force2Value.Text = sample.FastForce2N.ToString("0.000") + " N";
            _totalValue.Text = sample.FastForceTotalN.ToString("0.000") + " N";
            _windowLabel.Text =
                "Averaging window: " + sample.WindowCount + " / " + sample.WindowSize +
                "   |   Sample rate: " + sample.SampleRateHz.ToString("0") + " Hz";
        }

        private void SetStatus(string text, Color color)
        {
            _statusLabel.Text = text;
            _statusLabel.ForeColor = color;
            WriteLog(text);
        }

        private static void WriteLog(string text)
        {
            try
            {
                string path = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "QuantumXMonitor.log");
                File.AppendAllText(path, DateTime.Now.ToString("O") + " " + text + Environment.NewLine);
            }
            catch
            {
                // Logging must never interrupt measurement or shutdown.
            }
        }

        private static Color StatusColor(string status)
        {
            if (status.StartsWith("Connected", StringComparison.OrdinalIgnoreCase))
            {
                return Color.ForestGreen;
            }

            if (status.StartsWith("Zero complete", StringComparison.OrdinalIgnoreCase))
            {
                return Color.ForestGreen;
            }

            if (status.StartsWith("Overrange", StringComparison.OrdinalIgnoreCase) ||
                status.StartsWith("Zero failed", StringComparison.OrdinalIgnoreCase))
            {
                return Color.Firebrick;
            }

            return Color.DarkOrange;
        }

        private void PostToUi(Action action)
        {
            if (IsDisposed || Disposing)
            {
                return;
            }

            try
            {
                BeginInvoke(action);
            }
            catch (InvalidOperationException)
            {
                // The window was closed between the checks and BeginInvoke.
            }
        }

        private void OnFormClosing(object sender, FormClosingEventArgs eventArgs)
        {
            _forceServer.Dispose();
            if (_cancellation == null)
            {
                return;
            }

            _cancellation.Cancel();
            if (_readerTask != null)
            {
                try
                {
                    _readerTask.Wait(3000);
                }
                catch (AggregateException)
                {
                    // Any error has already been displayed by the continuation.
                }
            }

            _cancellation.Dispose();
        }
    }
}
