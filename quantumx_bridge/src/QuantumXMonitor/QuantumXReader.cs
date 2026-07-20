using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using Hbm.Api.Common;
using Hbm.Api.Common.Entities;
using Hbm.Api.Common.Entities.Channels;
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
        private const int FastAverageWindowSize = 20;
        private const int ZeroBalanceSampleCount = 100;
        private readonly string _ipAddress;
        private int _zeroBothRequested;

        public QuantumXReader(string ipAddress)
        {
            _ipAddress = ipAddress;
        }

        public event Action<string> StatusChanged;
        public event Action<FilteredSample> SampleReceived;

        public bool RequestZeroBoth()
        {
            return Interlocked.Exchange(ref _zeroBothRequested, 1) == 0;
        }

        public void Run(CancellationToken cancellationToken)
        {
            DaqEnvironment environment = null;
            QuantumXDevice device = null;
            DaqMeasurement measurement = null;
            bool daqStarted = false;

            try
            {
                OnStatus("Initializing Common API ...");
                environment = DaqEnvironment.GetInstance();
                WaitForApiInitialization(cancellationToken);

                OnStatus("Searching for MX440B ...");
                device = FindDevice(environment, _ipAddress);

                List<Problem> problems;
                if (!environment.Connect(device, out problems))
                {
                    throw new InvalidOperationException(FormatProblems("Connection failed", problems));
                }

                if (!string.Equals(device.Name, "MX440B", StringComparison.OrdinalIgnoreCase) &&
                    !string.Equals(device.Model, "MX440B", StringComparison.OrdinalIgnoreCase))
                {
                    throw new InvalidOperationException(
                        "Unexpected device: " + device.Model + " / " + device.Name);
                }

                Channel channel1 = GetPrimaryChannel(device, 1);
                Channel channel2 = GetPrimaryChannel(device, 2);
                Signal signal1 = GetPrimarySignal(channel1, 1);
                Signal signal2 = GetPrimarySignal(channel2, 2);
                string unit1 = device.GetUnit(channel1);
                string unit2 = device.GetUnit(channel2);
                if (!string.Equals(unit1, "N", StringComparison.OrdinalIgnoreCase) ||
                    !string.Equals(unit2, "N", StringComparison.OrdinalIgnoreCase))
                {
                    throw new InvalidOperationException(
                        "Both channels must be scaled in N (CH1=" + unit1 + ", CH2=" + unit2 + ").");
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
                    OnStatus("Fallback mode: single values - streaming is blocked by the network");
                    RunSingleValueLoop(device, channel1, channel2, signal1, signal2, cancellationToken);
                    return;
                }

                measurement.StartDaq(DataAcquisitionMode.TimestampSynchronized);
                daqStarted = true;

                OnStatus(
                    "Connected - firmware " + device.FirmwareVersion +
                    ", " + signal1.SampleRate.ToString("0") + " Hz");

                var average1 = new RollingAverage(AverageWindowSize);
                var average2 = new RollingAverage(AverageWindowSize);
                var averageTotal = new RollingAverage(AverageWindowSize);
                var fastAverage1 = new RollingAverage(FastAverageWindowSize);
                var fastAverage2 = new RollingAverage(FastAverageWindowSize);
                var fastAverageTotal = new RollingAverage(FastAverageWindowSize);
                double? measurementTimestampAnchor = null;
                long utcTimestampAnchorNs = 0;

                while (!cancellationToken.IsCancellationRequested)
                {
                    if (TakeZeroBothRequest())
                    {
                        OnStatus("Zeroing both sensors ...");
                        measurement.StopDaq();
                        daqStarted = false;
                        try
                        {
                            if (ZeroBothChannels(device, channel1, channel2))
                            {
                                ClearAverages(
                                    average1, average2, averageTotal,
                                    fastAverage1, fastAverage2, fastAverageTotal);
                            }
                        }
                        finally
                        {
                            measurement.StartDaq(DataAcquisitionMode.TimestampSynchronized);
                            daqStarted = true;
                            measurementTimestampAnchor = null;
                            utcTimestampAnchorNs = 0;
                        }
                    }

                    measurement.FillMeasurementValues(true);

                    int count1 = signal1.ContinuousMeasurementValues.UpdatedValueCount;
                    int count2 = signal2.ContinuousMeasurementValues.UpdatedValueCount;
                    int pairedCount = Math.Min(count1, count2);
                    bool overflow = false;

                    if (pairedCount > 0 && !measurementTimestampAnchor.HasValue)
                    {
                        measurementTimestampAnchor =
                            signal1.ContinuousMeasurementValues.Timestamps[pairedCount - 1];
                        utcTimestampAnchorNs = UtcNowNs();
                    }

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
                        fastAverage1.Add(force1);
                        fastAverage2.Add(force2);
                        fastAverageTotal.Add(force1 + force2);

                        double measurementTimestamp =
                            signal1.ContinuousMeasurementValues.Timestamps[index];
                        long timestampUtcNs = measurementTimestampAnchor.HasValue
                            ? utcTimestampAnchorNs + (long)Math.Round(
                                (measurementTimestamp - measurementTimestampAnchor.Value) * 1000000000.0)
                            : UtcNowNs();

                        // Publish one rolling mean per physical QuantumX sample.
                        // FillMeasurementValues still transfers data efficiently in blocks,
                        // while the original timestamps preserve the 2400 Hz time axis.
                        OnSample(new FilteredSample
                        {
                            Force1N = average1.Mean,
                            Force2N = average2.Mean,
                            ForceTotalN = averageTotal.Mean,
                            FastForce1N = fastAverage1.Mean,
                            FastForce2N = fastAverage2.Mean,
                            FastForceTotalN = fastAverageTotal.Mean,
                            RawForce1N = force1,
                            RawForce2N = force2,
                            RawForceTotalN = force1 + force2,
                            TimestampUtcNs = timestampUtcNs,
                            WindowCount = averageTotal.Count,
                            WindowSize = averageTotal.Capacity,
                            SampleRateHz = (double)signal1.SampleRate
                        });
                    }

                    if (overflow)
                    {
                        OnStatus("Overrange - invalid values are excluded from averaging");
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
            Channel channel1,
            Channel channel2,
            Signal signal1,
            Signal signal2,
            CancellationToken cancellationToken)
        {
            var signals = new List<Signal> { signal1, signal2 };
            var average1 = new RollingAverage(AverageWindowSize);
            var average2 = new RollingAverage(AverageWindowSize);
            var averageTotal = new RollingAverage(AverageWindowSize);
            var fastAverage1 = new RollingAverage(FastAverageWindowSize);
            var fastAverage2 = new RollingAverage(FastAverageWindowSize);
            var fastAverageTotal = new RollingAverage(FastAverageWindowSize);
            var rateTimer = Stopwatch.StartNew();
            double displayedRate = 0.0;

            while (!cancellationToken.IsCancellationRequested)
            {
                if (TakeZeroBothRequest())
                {
                    OnStatus("Zeroing both sensors ...");
                    if (ZeroBothChannels(device, channel1, channel2))
                    {
                        ClearAverages(
                            average1, average2, averageTotal,
                            fastAverage1, fastAverage2, fastAverageTotal);
                    }
                }

                device.ReadSingleMeasurementValue(signals);
                MeasurementValue value1 = signal1.GetSingleMeasurementValue();
                MeasurementValue value2 = signal2.GetSingleMeasurementValue();

                if (value1.State == MeasurementValueState.Valid &&
                    value2.State == MeasurementValueState.Valid)
                {
                    average1.Add(value1.Value);
                    average2.Add(value2.Value);
                    averageTotal.Add(value1.Value + value2.Value);
                    fastAverage1.Add(value1.Value);
                    fastAverage2.Add(value2.Value);
                    fastAverageTotal.Add(value1.Value + value2.Value);

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
                        FastForce1N = fastAverage1.Mean,
                        FastForce2N = fastAverage2.Mean,
                        FastForceTotalN = fastAverageTotal.Mean,
                        RawForce1N = value1.Value,
                        RawForce2N = value2.Value,
                        RawForceTotalN = value1.Value + value2.Value,
                        TimestampUtcNs = UtcNowNs(),
                        WindowCount = averageTotal.Count,
                        WindowSize = averageTotal.Capacity,
                        SampleRateHz = displayedRate
                    });
                }
                else
                {
                    OnStatus("Overrange - invalid values are excluded from averaging");
                }

                rateTimer.Restart();
                cancellationToken.WaitHandle.WaitOne(20);
            }
        }

        private static long UtcNowNs()
        {
            return (DateTime.UtcNow.Ticks - 621355968000000000L) * 100L;
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

            throw new InvalidOperationException("No MX440B found at " + expectedIp + ".");
        }

        private static Channel GetPrimaryChannel(QuantumXDevice device, int channelNumber)
        {
            int connectorIndex = channelNumber - 1;
            if (connectorIndex >= device.Connectors.Count ||
                device.Connectors[connectorIndex].Channels.Count == 0 ||
                device.Connectors[connectorIndex].Channels[0] == null)
            {
                throw new InvalidOperationException("Channel " + channelNumber + " is unavailable.");
            }

            return device.Connectors[connectorIndex].Channels[0];
        }

        private static Signal GetPrimarySignal(Channel channel, int channelNumber)
        {
            if (channel.Signals.Count == 0)
            {
                throw new InvalidOperationException("Channel " + channelNumber + " has no measurement signal.");
            }

            Signal signal = channel.Signals[0];
            if (!signal.IsMeasurable)
            {
                throw new InvalidOperationException("Channel " + channelNumber + " is not measurable.");
            }

            return signal;
        }

        private bool TakeZeroBothRequest()
        {
            return Interlocked.Exchange(ref _zeroBothRequested, 0) != 0;
        }

        private bool ZeroBothChannels(QuantumXDevice device, Channel channel1, Channel channel2)
        {
            var zero1 = channel1 as IZero;
            var zero2 = channel2 as IZero;
            if (zero1 == null || zero2 == null || zero1.Zero == null || zero2.Zero == null)
            {
                OnStatus("Zero failed: both channels must support zero balancing.");
                return false;
            }
            if (zero1.Zero.IsZeroBalancingInhibited || zero2.Zero.IsZeroBalancingInhibited)
            {
                OnStatus("Zero failed: zero balancing is disabled for at least one channel.");
                return false;
            }

            zero1.Zero.Target = 0.0;
            zero2.Zero.Target = 0.0;
            try
            {
                List<Problem> problems;
                bool success = device.SetZeroBalance(
                    new List<Channel> { channel1, channel2 },
                    out problems,
                    ZeroBalanceSampleCount);
                if (!success)
                {
                    OnStatus(FormatProblems("Zero failed", problems));
                    return false;
                }
            }
            catch (Exception exception)
            {
                OnStatus("Zero failed: " + exception.Message);
                return false;
            }

            OnStatus("Zero complete - both sensors set to 0 N");
            return true;
        }

        private static void ClearAverages(params RollingAverage[] averages)
        {
            foreach (RollingAverage average in averages)
            {
                average.Clear();
            }
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
