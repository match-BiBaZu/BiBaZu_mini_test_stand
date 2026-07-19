using System;

namespace QuantumXMonitor
{
    internal sealed class RollingAverage
    {
        private readonly double[] _values;
        private int _next;
        private int _count;
        private double _sum;

        public RollingAverage(int capacity)
        {
            if (capacity <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(capacity));
            }

            _values = new double[capacity];
        }

        public int Count => _count;
        public int Capacity => _values.Length;
        public double Mean => _count == 0 ? double.NaN : _sum / _count;

        public void Add(double value)
        {
            if (_count == _values.Length)
            {
                _sum -= _values[_next];
            }
            else
            {
                _count++;
            }

            _values[_next] = value;
            _sum += value;
            _next = (_next + 1) % _values.Length;
        }
    }
}
