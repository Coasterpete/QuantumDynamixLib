using System;
using System.Threading.Tasks;
using Avalonia.Automation;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Interactivity;
using QuantumDynamix.Math;

namespace QuantumDynamix.App;

internal sealed partial class WorkspaceView
{
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

    // "Adjoining" keeps the adjoining Region's type, as the Region authoring rules already do.
    private static readonly string[] NewRegionModes = ["Adjoining", "Coordinate accel.", "Force-based", "Geometric"];

    private void AppendRegion(object? sender, RoutedEventArgs e) => CreateRegion(prepend: false);
    private void PrependRegion(object? sender, RoutedEventArgs e) => CreateRegion(prepend: true);

    private void AppendRegionOnEnter(object? sender, KeyEventArgs e)
    {
        if (e.Key == Key.Enter && AppendRegionButton.IsEnabled)
        {
            CreateRegion(prepend: false);
            e.Handled = true;
        }
    }

    private void RefreshNewRegionControls()
    {
        TimelineEditorSession? session = IsTrack ? track!.Timeline : null;
        NewRegionPanel.IsVisible = session is not null;
        if (session is null) { return; }
        NewRegionModeLabel.IsVisible = NewRegionMode.IsVisible = session.SupportsForceRegions;
        bool first = session.Regions.Count == 0;
        FirstStationLabel.IsVisible = FirstStationInput.IsVisible = FirstStationUnit.IsVisible = first;
        NewRegionLength.PlaceholderText = first ? "required" : "adjoining";
        string tip = first ? "Heartline length of the first Region."
            : "Leave blank to use the adjoining Region's length. Prepend uses at most the space before the first Region.";
        ToolTip.SetTip(NewRegionLength, tip);
        AutomationProperties.SetHelpText(NewRegionLength, tip);
    }

    /// <summary>Appends or prepends directly in the editor. The Region appears immediately as one Undo/Redo action.</summary>
    private void CreateRegion(bool prepend)
    {
        if (!IsTrack) { return; }
        graph.CancelDrag();
        TimelineEditorSession session = track!.Timeline;
        var regions = session.Regions;
        AuthoredRegion? neighbor = regions.Count == 0 ? null : prepend ? regions[0] : regions[^1];
        RegionAuthoringMode? mode = NewRegionMode.IsVisible ? NewRegionMode.SelectedIndex switch
        {
            1 => RegionAuthoringMode.CoordinateAcceleration,
            2 => RegionAuthoringMode.Force,
            3 => RegionAuthoringMode.Geometric,
            _ => null
        } : null;

        double length;
        double firstStation = 0;
        if (string.IsNullOrWhiteSpace(NewRegionLength.Text) && neighbor is not null)
        {
            length = neighbor.Definition.End - neighbor.Definition.Start;
            if (prepend) { length = System.Math.Min(length, neighbor.Definition.Start); }
        }
        else if (!TryReadNumber(NewRegionLength.Text, out length))
        {
            StatusLabel.Text = neighbor is null ? "Enter the first Region's heartline length."
                : "Enter a finite, positive heartline length, or leave Length blank to use the adjoining Region's length.";
            NewRegionLength.Focus();
            return;
        }
        if (neighbor is null && !TryReadNumber(FirstStationInput.Text, out firstStation))
        {
            StatusLabel.Text = "Enter a finite, nonnegative first station.";
            FirstStationInput.Focus();
            return;
        }

        string? error = session.AddRegion(prepend, length, firstStation, mode);
        if (error is not null)
        {
            StatusLabel.Text = prepend && neighbor is not null
                ? $"{error} {Format(neighbor.Definition.Start)} m is available before {neighbor.Name}." : error;
            return;
        }
        int index = prepend ? 0 : session.Regions.Count - 1;
        AuthoredRegion added = session.Regions[index];
        // A blank length then repeats this Region's length through the adjoining-length default.
        NewRegionLength.Text = "";
        RefreshRegions(added.Id, index, EditingChannel);
        StatusLabel.Text = $"Added {added.Name} · {DescribeMode(added.Mode)} · {DescribeExtent(added.Definition)}.";
    }

    private void EditRegion(object? sender, RoutedEventArgs e)
    {
        if (!IsTrack || SelectedRegion < 0) { return; }
        SetInspectorMode(InspectorMode.Region);
        KeyframePanel.ScrollToHome();
        RegionNameInput.Focus();
        RegionNameInput.SelectAll();
    }

    private void ShowRegionFields(object? sender, RoutedEventArgs e) => SetInspectorMode(InspectorMode.Region);

    private void ApplyRegionOnEnter(object? sender, KeyEventArgs e)
    {
        if (e.Key == Key.Enter)
        {
            ApplyRegionFields(sender, e);
            e.Handled = true;
        }
    }

    private void RefreshRegionFields()
    {
        if (!IsTrack || SelectedRegion < 0) { return; }
        AuthoredRegion region = track!.Timeline.Regions[SelectedRegion];
        RegionSummary.Text = $"{DescribeMode(region.Mode)} · {Format(region.Definition.Start)}–{Format(region.Definition.End)} m";
        RegionNameInput.Text = region.Name;
        RegionLengthInput.Text = Format(region.Definition.End - region.Definition.Start);
    }

    private void ApplyRegionFields(object? sender, RoutedEventArgs e)
    {
        if (!IsTrack || SelectedRegion < 0) { return; }
        int index = SelectedRegion;
        TimelineEditorSession session = track!.Timeline;
        AuthoredRegion region = session.Regions[index];
        if (!TryReadNumber(RegionLengthInput.Text, out double length))
        {
            StatusLabel.Text = "Enter a finite, positive heartline length.";
            return;
        }
        string? error = session.UpdateRegion(index, RegionNameInput.Text ?? "", length);
        if (error is null)
        {
            RefreshRegions(region.Id, index, EditingChannel);
            StatusLabel.Text = $"Updated {session.Regions[index].Name} · {DescribeExtent(session.Regions[index].Definition)}.";
        }
        else
        {
            StatusLabel.Text = error;
        }
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
