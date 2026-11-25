using System;
using Avalonia;

namespace QuantumDynamix.App;

internal static class Program
{
    [STAThread]
    private static int Main(string[] args)
    {
        return AppBuilder.Configure<App>()
            .UsePlatformDetect()
            .StartWithClassicDesktopLifetime(args);
    }
}
