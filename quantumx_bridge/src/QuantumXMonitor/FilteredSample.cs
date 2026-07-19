namespace QuantumXMonitor
{
    internal sealed class FilteredSample
    {
        public double Force1N { get; set; }
        public double Force2N { get; set; }
        public double ForceTotalN { get; set; }
        public int WindowCount { get; set; }
        public int WindowSize { get; set; }
        public double SampleRateHz { get; set; }
    }
}
