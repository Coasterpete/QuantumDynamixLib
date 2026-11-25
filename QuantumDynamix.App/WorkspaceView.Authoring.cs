using System;
using System.Threading.Tasks;
using Avalonia.Automation;
using Avalonia.Controls;
using Avalonia.Interactivity;
using QuantumDynamix.Math;

namespace QuantumDynamix.App;

internal sealed partial class WorkspaceView
{
    private const string RegionName = "Region name";
    private const string RegionLength = "Heartline length (m)";
    private const string FirstStation = "First station (m)";
    private const string ForwardSpeed = "Prescribed forward speed (m/s)";
    private const string StartX = "Start X (m)";
    private const string StartY = "Start Y (m)";
    private const string StartZ = "Start Z (m)";
    private const string Heading = "Start heading (degrees from +X toward +Y)";
    private const string Pitch = "Start pitch (degrees above horizontal)";
    private const string ChannelName = "Channel name";
    private const string StartValue = "Start value";
    private const string EndValue = "End value";
    private const string CoordinateAccelerationRegion = "Coordinate-acceleration Region";
    private const string ForceBasedRegion = "Force-based Region";
    private const string GeometricRegion = "Geometric Region";

    private static string DescribeMode(RegionAuthoringMode mode) => mode switch
    {
        RegionAuthoringMode.Force => ForceBasedRegion,
        RegionAuthoringMode.Geometric => GeometricRegion,
        _ => CoordinateAccelerationRegion
    };

    private async void NewTrack(object? sender, RoutedEventArgs e) =>
        await RunProjectOperation(CreateTrack);

    private async Task CreateTrack()
    {
        const string motion = "The prescribed speed stays constant during geometry reconstruction. The wireframe uses the reference spine and rail offsets.";
        TrackDocument? created = null;
        await ShowForm("New Track", "Create", motion, form =>
        {
            string name = form.Read("Track name");
            if (name.Length == 0)
            {
                return "Enter a track name.";
            }
            if (!TryReadNumber(form.Read(ForwardSpeed), out double speed) || !RegionTrackGeometry.SupportsSpeed(speed))
            {
                return "Enter a finite forward speed of at least 0.0001 m/s.";
            }
            if (!TryReadNumber(form.Read(StartX), out double x) || !TryReadNumber(form.Read(StartY), out double y) ||
                !TryReadNumber(form.Read(StartZ), out double z) || !TryReadNumber(form.Read(Heading), out double heading) ||
                !TryReadNumber(form.Read(Pitch), out double pitch) || pitch is < -90 or > 90)
            {
                return "Enter a finite start position and heading, and a pitch between −90° and 90°.";
            }
            const double radiansPerDegree = System.Math.PI / 180;
            Quaterniond orientation = Quaterniond.FromAxisAngle(Vector3d.UnitZ, (heading % 360) * radiansPerDegree)
                * Quaterniond.FromAxisAngle(Vector3d.UnitY, -pitch * radiansPerDegree);
            created = TrackDocument.CreateAuthored(name, new(new(x, y, z), orientation), speed);
            return null;
        }, ("Track name", "Untitled Track"), (ForwardSpeed, "10"), (StartX, "0"), (StartY, "0"), (StartZ, "0"),
            (Heading, "0"), (Pitch, "0"));
        if (created is not null && await ConfirmReplaceProject())
        {
            OpenTrack(created);
        }
    }

    private async void AppendRegion(object? sender, RoutedEventArgs e) => await CreateRegion(prepend: false);
    private async void PrependRegion(object? sender, RoutedEventArgs e) => await CreateRegion(prepend: true);

    private Task CreateRegion(bool prepend)
    {
        TimelineEditorSession session = track!.Timeline;
        var regions = session.Regions;
        bool firstRegion = regions.Count == 0;
        AuthoredRegion? neighbor = firstRegion ? null : prepend ? regions[0] : regions[^1];
        string title = prepend ? "Prepend Region" : "Append Region";
        string context = neighbor is null ? "Starts straight with zero banking. Set the first station above 0 m to leave room to prepend."
            : prepend ? $"Before {neighbor.Name} · {Format(neighbor.Definition.Start)} m available."
            : $"After {neighbor.Name} · joins at {Format(neighbor.Definition.End)} m.";
        ComboBox? mode = null;
        if (session.SupportsForceRegions)
        {
            mode = new ComboBox
            {
                ItemsSource = new[] { CoordinateAccelerationRegion, ForceBasedRegion, GeometricRegion },
                SelectedItem = DescribeMode(neighbor?.Mode ?? RegionAuthoringMode.CoordinateAcceleration),
                HorizontalAlignment = Avalonia.Layout.HorizontalAlignment.Stretch
            };
            AutomationProperties.SetName(mode, "Region mode");
        }
        return new AuthoringDialog(title, "Create", context, form =>
        {
            if (!TryReadNumber(form.Read(RegionLength), out double length))
            {
                return "Enter a finite, positive heartline length.";
            }
            double firstStation = 0;
            if (firstRegion && !TryReadNumber(form.Read(FirstStation), out firstStation))
            {
                return "Enter a finite, nonnegative first station.";
            }

            RegionAuthoringMode selectedMode = mode?.SelectedItem switch
            {
                ForceBasedRegion => RegionAuthoringMode.Force,
                GeometricRegion => RegionAuthoringMode.Geometric,
                _ => RegionAuthoringMode.CoordinateAcceleration
            };
            string? error = session.AddRegion(prepend, length, firstStation, selectedMode);
            if (error is null)
            {
                int index = prepend ? 0 : session.Regions.Count - 1;
                RefreshRegions(session.Regions[index].Id, index, EditingChannel);
                StatusLabel.Text = $"Added {session.Regions[index].Name}.";
            }
            return error;
        }, firstRegion ? [(RegionLength, ""), (FirstStation, "0")] : [(RegionLength, "")], mode)
            .ShowDialog((Window)TopLevel.GetTopLevel(this)!);
    }

    private async void EditRegion(object? sender, RoutedEventArgs e)
    {
        int index = SelectedRegion;
        var session = track!.Timeline;
        AuthoredRegion region = session.Regions[index];
        string range = $"{Format(region.Definition.Start)}–{Format(region.Definition.End)} m";
        if (session.SupportsForceRegions) { range = $"{DescribeMode(region.Mode)} · {range}"; }
        await ShowForm("Region Properties", "Apply", range, form =>
        {
            if (!TryReadNumber(form.Read(RegionLength), out double length))
            {
                return "Enter a finite, positive heartline length.";
            }
            string? error = session.UpdateRegion(index, form.Read(RegionName), length);
            if (error is null)
            {
                RefreshRegions(region.Id, index, EditingChannel);
                StatusLabel.Text = $"Updated {session.Regions[index].Name}.";
            }
            return error;
        }, (RegionName, region.Name), (RegionLength, Format(region.Definition.End - region.Definition.Start)));
    }

    private async void AddChannel(object? sender, RoutedEventArgs e)
    {
        int index = SelectedRegion;
        var session = track!.Timeline;
        AuthoredRegion region = session.Regions[index];
        await ShowForm("Add Channel", "Add", $"Region: {region.Name}. Custom channels appear in the Transition Editor only.", form =>
        {
            if (!TryReadNumber(form.Read(StartValue), out double from) || !TryReadNumber(form.Read(EndValue), out double to))
            {
                return "Enter finite numbers for both endpoint values.";
            }
            string channel = form.Read(ChannelName);
            string? error = session.AddChannel(index, channel, from, to);
            if (error is null)
            {
                RefreshRegions(region.Id, index, channel);
                StatusLabel.Text = $"Added the {channel} channel.";
            }
            return error;
        }, (ChannelName, ""), (StartValue, ""), (EndValue, ""));
    }

    private Task ShowForm(string title, string action, string? message, Func<AuthoringDialog, string?> commit,
        params (string Label, string Initial)[] fields) =>
        new AuthoringDialog(title, action, message, commit, fields).ShowDialog((Window)TopLevel.GetTopLevel(this)!);

    private void DeleteRegion(object? sender, RoutedEventArgs e)
    {
        int index = SelectedRegion;
        var session = track!.Timeline;
        string name = session.Regions[index].Name;
        string? error = session.DeleteRegion(index);
        if (error is null)
        {
            RefreshRegions(null, index, EditingChannel);
        }
        StatusLabel.Text = error ?? $"Deleted {name}.";
    }
}
