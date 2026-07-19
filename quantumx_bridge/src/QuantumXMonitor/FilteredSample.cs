namespace QuantumXMonitor
{
    internal sealed class FilteredSample
    {
        public double Force1N { get; set; }
        public double Force2N { get; set; }
        public double ForceTotalN { get; set; }
        public double FastForce1N { get; set; }
        public double FastForce2N { get; set; }
        public double FastForceTotalN { get; set; }
        public double RawForce1N { get; set; }
        public double RawForce2N { get; set; }
        public double RawForceTotalN { get; set; }
        public long TimestampUtcNs { get; set; }
        public int WindowCount { get; set; }
        public int WindowSize { get; set; }
        public double SampleRateHz { get; set; }
    }
}
