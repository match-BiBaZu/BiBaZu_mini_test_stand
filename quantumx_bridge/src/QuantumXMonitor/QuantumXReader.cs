using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using Hbm.Api.Common;
using Hbm.Api.Common.Entities;
using Hbm.Api.Common.Exceptions;
using Hbm.Api.Common.Entities.ConnectionInfos;
using Hbm.Api.Common.Entities.Problems;
using Hbm.Api.Common.Entities.Signals;
using Hbm.Api.Common.Enums;
using Hbm.Api.QuantumX;

namespace QuantumXMonitor
{
    internal sealed class QuantumXReader
    {
        private const int AverageWindowSize = 100;
        private readonly string _ipAddress;

        public QuantumXReader(string ipAddress)
        {
            _ipAddress = ipAddress;
        }

        public event Action<string> StatusChanged;
        public event Action<FilteredSample> SampleReceived;

        public void Run(CancellationToken cancellationToken)
        {
            DaqEnvironment environment = null;
            QuantumXDevice device = null;
            DaqMeasurement measurement = null;
            bool daqStarted = false;

            try
            {
                OnStatus("Common API wird initialisiert …");
                environment = DaqEnvironment.GetInstance();
                WaitForApiInitialization(cancellationToken);

                OnStatus("MX440B wird gesucht …");
                device = FindDevice(environment, _ipAddress);

                List<Problem> problems;
                if (!environment.Connect(device, out problems))
                {
                    throw new InvalidOperationException(FormatProblems("Verbindung fehlgeschlagen", problems));
                }

                if (!string.Equals(device.Name, "MX440B", StringComparison.OrdinalIgnoreCase) &&
                    !string.Equals(device.Model, "MX440B", StringComparison.OrdinalIgnoreCase))
                {
                    throw new InvalidOperationException(
                        "Unerwartetes Gerät: " + device.Model + " / " + device.Name);
                }

                Signal signal1 = GetPrimarySignal(device, 1);
                Signal signal2 = GetPrimarySignal(device, 2);
                string unit1 = device.GetUnit(device.Connectors[0].Channels[0]);
                string unit2 = device.GetUnit(device.Connectors[1].Channels[0]);
                if (!string.Equals(unit1, "N", StringComparison.OrdinalIgnoreCase) ||
                    !string.Equals(unit2, "N", StringComparison.OrdinalIgnoreCase))
                {
                    throw new InvalidOperationException(
                        "Beide Kanäle müssen in N skaliert sein (K1=" + unit1 + ", K2=" + unit2 + ").");
                }

                measurement = new DaqMeasurement();
                measurement.AddSignals(device, new List<Signal> { signal1, signal2 });
                try
                {
                    // Use the parameter set and overloads from HBK's installed
                    // Hbm.Api.DemoProject for continuous QuantumX acquisition.
                    measurement.PrepareDaq(5000, 10, 1000, false, false);
                }
                catch (StreamingInitializationFailedException)
                {
                    measurement.Dispose();
                    measurement = null;
                    OnStatus("Ersatzmodus: Einzelwerte – Streaming ist im Netzwerk blockiert");
                    RunSingleValueLoop(device, signal1, signal2, cancellationToken);
                    return;
                }

                measurement.StartDaq(DataAcquisitionMode.TimestampSynchronized);
                daqStarted = true;

                OnStatus(
                    "Verbunden – Firmware " + device.FirmwareVersion +
                    ", " + signal1.SampleRate.ToString("0") + " Hz");

                var average1 = new RollingAverage(AverageWindowSize);
                var average2 = new RollingAverage(AverageWindowSize);
                var averageTotal = new RollingAverage(AverageWindowSize);

                while (!cancellationToken.IsCancellationRequested)
                {
                    measurement.FillMeasurementValues(true);

                    int count1 = signal1.ContinuousMeasurementValues.UpdatedValueCount;
                    int count2 = signal2.ContinuousMeasurementValues.UpdatedValueCount;
                    int pairedCount = Math.Min(count1, count2);
                    bool overflow = false;

                    for (int index = 0; index < pairedCount; index++)
                    {
                        if (signal1.ContinuousMeasurementValues.States[index] != MeasurementValueState.Valid ||
                            signal2.ContinuousMeasurementValues.States[index] != MeasurementValueState.Valid)
                        {
                            overflow = true;
                            continue;
                        }

                        double force1 = signal1.ContinuousMeasurementValues.Values[index];
                        double force2 = signal2.ContinuousMeasurementValues.Values[index];
                        average1.Add(force1);
                        average2.Add(force2);
                        averageTotal.Add(force1 + force2);
                    }

                    if (overflow)
                    {
                        OnStatus("Overrange – ungültige Werte werden nicht gemittelt");
                    }

                    if (pairedCount > 0 && averageTotal.Count > 0)
                    {
                        OnSample(new FilteredSample
                        {
                            Force1N = average1.Mean,
                            Force2N = average2.Mean,
                            ForceTotalN = averageTotal.Mean,
                            WindowCount = averageTotal.Count,
                            WindowSize = averageTotal.Capacity,
                            SampleRateHz = (double)signal1.SampleRate
                        });
                    }

                    cancellationToken.WaitHandle.WaitOne(20);
                }
            }
            finally
            {
                if (measurement != null)
                {
                    if (daqStarted)
                    {
                        try
                        {
                            measurement.StopDaq();
                        }
                        catch
                        {
                            // Continue cleanup after a communication loss.
                        }
                    }

                    measurement.Dispose();
                }

                if (environment != null && device != null && device.IsConnected)
                {
                    try
                    {
                        environment.Disconnect(device);
                    }
                    catch
                    {
                        // The device can already be gone after a cable interruption.
                    }
                }

                if (environment != null)
                {
                    environment.Dispose();
                }
            }
        }

        private void RunSingleValueLoop(
            QuantumXDevice device,
            Signal signal1,
            Signal signal2,
            CancellationToken cancellationToken)
        {
            var signals = new List<Signal> { signal1, signal2 };
            var average1 = new RollingAverage(AverageWindowSize);
            var average2 = new RollingAverage(AverageWindowSize);
            var averageTotal = new RollingAverage(AverageWindowSize);
            var rateTimer = Stopwatch.StartNew();
            double displayedRate = 0.0;

            while (!cancellationToken.IsCancellationRequested)
            {
                device.ReadSingleMeasurementValue(signals);
                MeasurementValue value1 = signal1.GetSingleMeasurementValue();
                MeasurementValue value2 = signal2.GetSingleMeasurementValue();

                if (value1.State == MeasurementValueState.Valid &&
                    value2.State == MeasurementValueState.Valid)
                {
                    average1.Add(value1.Value);
                    average2.Add(value2.Value);
                    averageTotal.Add(value1.Value + value2.Value);

                    if (rateTimer.Elapsed.TotalSeconds > 0.0)
                    {
                        double currentRate = 1.0 / rateTimer.Elapsed.TotalSeconds;
                        displayedRate = displayedRate == 0.0
                            ? currentRate
                            : (0.9 * displayedRate) + (0.1 * currentRate);
                    }

                    OnSample(new FilteredSample
                    {
                        Force1N = average1.Mean,
                        Force2N = average2.Mean,
                        ForceTotalN = averageTotal.Mean,
                        WindowCount = averageTotal.Count,
                        WindowSize = averageTotal.Capacity,
                        SampleRateHz = displayedRate
                    });
                }
                else
                {
                    OnStatus("Overrange – ungültige Werte werden nicht gemittelt");
                }

                rateTimer.Restart();
                cancellationToken.WaitHandle.WaitOne(20);
            }
        }

        private static void WaitForApiInitialization(CancellationToken cancellationToken)
        {
            if (cancellationToken.WaitHandle.WaitOne(6500))
            {
                cancellationToken.ThrowIfCancellationRequested();
            }
        }

        private static QuantumXDevice FindDevice(DaqEnvironment environment, string expectedIp)
        {
            List<Device> devices = environment.Scan(new List<string> { "QuantumX" });
            foreach (Device scannedDevice in devices)
            {
                var ethernet = scannedDevice.ConnectionInfo as EthernetConnectionInfo;
                if (ethernet != null &&
                    string.Equals(ethernet.IpAddress, expectedIp, StringComparison.OrdinalIgnoreCase))
                {
                    var quantumX = scannedDevice as QuantumXDevice;
                    if (quantumX != null)
                    {
                        return quantumX;
                    }
                }
            }

            throw new InvalidOperationException("Keine MX440B unter " + expectedIp + " gefunden.");
        }

        private static Signal GetPrimarySignal(QuantumXDevice device, int channelNumber)
        {
            int connectorIndex = channelNumber - 1;
            if (connectorIndex >= device.Connectors.Count ||
                device.Connectors[connectorIndex].Channels.Count == 0 ||
                device.Connectors[connectorIndex].Channels[0] == null ||
                device.Connectors[connectorIndex].Channels[0].Signals.Count == 0)
            {
                throw new InvalidOperationException("Kanal " + channelNumber + " ist nicht verfügbar.");
            }

            Signal signal = device.Connectors[connectorIndex].Channels[0].Signals[0];
            if (!signal.IsMeasurable)
            {
                throw new InvalidOperationException("Kanal " + channelNumber + " ist nicht messbar.");
            }

            return signal;
        }

        private static string FormatProblems(string prefix, IEnumerable<Problem> problems)
        {
            string details = problems == null
                ? string.Empty
                : string.Join("; ", problems.Select(problem => problem.Message));
            return string.IsNullOrWhiteSpace(details) ? prefix : prefix + ": " + details;
        }

        private void OnStatus(string status)
        {
            StatusChanged?.Invoke(status);
        }

        private void OnSample(FilteredSample sample)
        {
            SampleReceived?.Invoke(sample);
        }
    }
}
