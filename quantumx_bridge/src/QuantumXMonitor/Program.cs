using System;
using System.Windows.Forms;

namespace QuantumXMonitor
{
    internal static class Program
    {
        [STAThread]
        private static void Main(string[] args)
        {
            bool serverOnly = Array.Exists(
                args,
                argument => string.Equals(argument, "--server-only", StringComparison.OrdinalIgnoreCase));
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new MonitorForm(serverOnly));
        }
    }
}
