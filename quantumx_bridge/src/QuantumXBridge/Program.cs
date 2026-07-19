using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Threading;
using Hbm.Api.Common;
using Hbm.Api.Common.Entities;
using Hbm.Api.Common.Entities.Channels;
using Hbm.Api.Common.Entities.Problems;
using Hbm.Api.Common.Entities.Signals;
using Hbm.Api.Common.Enums;
using Hbm.Api.QuantumX;
using Newtonsoft.Json;

namespace QuantumXBridge
{
    internal static class Program
    {
        private const string DefaultIp = "192.168.10.20";

        private static int Main(string[] args)
        {
            string ip = args.Length > 0 ? args[0] : DefaultIp;
            int sampleCount = args.Length > 1 ? ParsePositiveInt(args[1], "sample count") : 20;
            int intervalMs = args.Length > 2 ? ParsePositiveInt(args[2], "interval milliseconds") : 100;

            DaqEnvironment environment = null;
            QuantumXDevice device = null;

            try
            {
                environment = DaqEnvironment.GetInstance();
                device = new QuantumXDevice(ip);

                List<Problem> problems;
                if (!environment.Connect(device, out problems))
                {
                    WriteProblems("Connection failed", problems);
                    return 2;
                }

                WriteProblems("Connection warnings", problems);
                Console.Error.WriteLine(
                    "Connected: model={0}, name={1}, serial={2}, firmware={3}, connectors={4}",
                    device.Model,
                    device.Name,
                    device.SerialNo,
                    device.FirmwareVersion,
                    device.Connectors.Count);

                if (!string.Equals(device.Model, "MX440B", StringComparison.OrdinalIgnoreCase))
                {
                    Console.Error.WriteLine("Expected MX440B but connected to '{0}'.", device.Model);
                    return 3;
                }

                Signal force1 = GetPrimarySignal(device, 1);
                Signal force2 = GetPrimarySignal(device, 2);
                PrintSignalInfo(device, 1, force1);
                PrintSignalInfo(device, 2, force2);

                var signals = new List<Signal> { force1, force2 };
                for (long sequence = 1; sequence <= sampleCount; sequence++)
                {
                    device.ReadSingleMeasurementValue(signals);

                    MeasurementValue value1 = force1.GetSingleMeasurementValue();
                    MeasurementValue value2 = force2.GetSingleMeasurementValue();
                    bool valid1 = value1.State == MeasurementValueState.Valid;
                    bool valid2 = value2.State == MeasurementValueState.Valid;

                    double? force1N = valid1 ? (double?)value1.Value : null;
                    double? force2N = valid2 ? (double?)value2.Value : null;
                    double? totalN = valid1 && valid2 ? (double?)(value1.Value + value2.Value) : null;

                    var sample = new
                    {
                        schema_version = 1,
                        sequence,
                        timestamp_utc_ns = UtcNowNanoseconds(),
                        force_1_n = force1N,
                        force_2_n = force2N,
                        force_total_n = totalN,
                        status = valid1 && valid2 ? "ok" : "overrange",
                        channel_1_status = MapState(value1.State),
                        channel_2_status = MapState(value2.State),
                        api_timestamp_1_s = value1.Timestamp,
                        api_timestamp_2_s = value2.Timestamp
                    };

                    Console.WriteLine(JsonConvert.SerializeObject(sample, Formatting.None));
                    if (sequence < sampleCount)
                    {
                        Thread.Sleep(intervalMs);
                    }
                }

                return 0;
            }
            catch (Exception exception)
            {
                Console.Error.WriteLine("{0}: {1}", exception.GetType().FullName, exception.Message);
                Console.Error.WriteLine(exception.StackTrace);
                return 1;
            }
            finally
            {
                if (environment != null && device != null && device.IsConnected)
                {
                    try
                    {
                        environment.Disconnect(device);
                    }
                    catch (Exception exception)
                    {
                        Console.Error.WriteLine("Disconnect failed: {0}", exception.Message);
                    }
                }

                if (environment != null)
                {
                    environment.Dispose();
                }
            }
        }

        private static Signal GetPrimarySignal(QuantumXDevice device, int channelNumber)
        {
            int connectorIndex = channelNumber - 1;
            if (connectorIndex < 0 || connectorIndex >= device.Connectors.Count)
            {
                throw new InvalidOperationException("Connector for channel " + channelNumber + " is unavailable.");
            }

            var connector = device.Connectors[connectorIndex];
            if (connector.Channels == null || connector.Channels.Count == 0 || connector.Channels[0] == null)
            {
                throw new InvalidOperationException("Channel " + channelNumber + " is unavailable.");
            }

            var channel = connector.Channels[0];
            if (channel.Signals == null || channel.Signals.Count == 0 || channel.Signals[0] == null)
            {
                throw new InvalidOperationException("Signal for channel " + channelNumber + " is unavailable.");
            }

            Signal signal = channel.Signals[0];
            if (!signal.IsMeasurable)
            {
                throw new InvalidOperationException("Signal for channel " + channelNumber + " is not measurable.");
            }

            return signal;
        }

        private static void PrintSignalInfo(QuantumXDevice device, int channelNumber, Signal signal)
        {
            Channel channel = device.Connectors[channelNumber - 1].Channels[0];
            string unit = channel is IUnit unitChannel ? unitChannel.Unit : "<unknown>";
            Console.Error.WriteLine(
                "Channel {0}: channel='{1}', signal='{2}', unit='{3}', sample_rate={4} Hz, id='{5}'",
                channelNumber,
                channel.Name,
                signal.Name,
                unit,
                signal.HasSampleRate ? signal.SampleRate.ToString(CultureInfo.InvariantCulture) : "n/a",
                signal.GetUniqueID());
        }

        private static string MapState(MeasurementValueState state)
        {
            return state == MeasurementValueState.Valid ? "ok" : "overrange";
        }

        private static long UtcNowNanoseconds()
        {
            return (DateTime.UtcNow.Ticks - 621355968000000000L) * 100L;
        }

        private static int ParsePositiveInt(string value, string name)
        {
            int parsed;
            if (!int.TryParse(value, NumberStyles.Integer, CultureInfo.InvariantCulture, out parsed) || parsed <= 0)
            {
                throw new ArgumentException(name + " must be a positive integer.");
            }

            return parsed;
        }

        private static void WriteProblems(string prefix, IEnumerable<Problem> problems)
        {
            if (problems == null)
            {
                return;
            }

            foreach (Problem problem in problems)
            {
                Console.Error.WriteLine("{0}: {1}", prefix, problem.Message);
            }
        }
    }
}
