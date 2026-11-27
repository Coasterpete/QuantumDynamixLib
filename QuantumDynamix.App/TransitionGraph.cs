using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Media;
using Avalonia.Media.Immutable;
using QuantumDynamix.Math.Timelines;

namespace QuantumDynamix.App;

internal sealed partial class TransitionGraph : Control
{
    private static readonly IBrush GridDot = new ImmutableSolidColorBrush(Color.Parse("#5F5F5F"));
    private static readonly IBrush MajorGridDot = new ImmutableSolidColorBrush(Color.Parse("#AEAEAE"));
    private static readonly IBrush Label = new ImmutableSolidColorBrush(Color.Parse("#D4DCE7"));
    private static readonly ImmutablePen OuterBoundary = new(new ImmutableSolidColorBrush(Color.Parse("#777777")), 1);
    private static readonly ImmutablePen InnerBoundary = new(new ImmutableSolidColorBrush(Color.Parse("#BBBBBB")), 1,
        new ImmutableDashStyle([4, 4], 0));
    private static readonly ImmutablePen Ruler = new(new ImmutableSolidColorBrush(Color.Parse("#9BA9B8")), 1);
    private static readonly ImmutableSolidColorBrush Highlight = new(Color.Parse("#FF0808"));
    private static readonly ImmutablePen SelectionLine = new(Highlight, 1.5);
    private static readonly ImmutablePen SelectedCurve = new(Brushes.White, 1);
    private static readonly IBrush LabelBackground = new ImmutableSolidColorBrush(Color.Parse("#E6000000"));
    private static readonly Typeface GraphType = new("Cascadia Mono, Consolas, DejaVu Sans Mono, monospace");
    private static readonly string[] ExampleChannels = ["Blue", "Red", "Green", "Purple", "Gold"];
    private static readonly ImmutablePen[] CurvePens =
    [
        CurvePen("#2400FF"), CurvePen("#FF0808"), CurvePen("#97FF00"), CurvePen("#8000FF"), CurvePen("#FFB800"),
        CurvePen("#00DAF5"), CurvePen("#FF80C0"), CurvePen("#EAFF70")
    ];

    private readonly TransitionEditState edit = new();
    private readonly Func<GraphPosition, TransitionDragTarget?>? resolveDragTarget;
    private readonly HashSet<string> hiddenChannels = new(StringComparer.Ordinal);
    private readonly Dictionary<string, int> channelColors = new(StringComparer.Ordinal);
    private string[] channels = [];
    private IReadOnlyList<Region> regions;
    private IReadOnlyList<Region>? previewRegions;
    private IReadOnlyList<string>? regionNames;
    private IReadOnlyList<TimelineKeyframe> keys = [];
    private double? viewStart;
    private double? viewEnd;
    private double? domainStart;
    private double? domainEnd;
    private double minimumValue;
    private double maximumValue = 1;
    private string? editChannel;
    private int activeRegion = -1;
    private int selectedKey = -1;
    private int selectedTransition = -1;
    private string? selectedTransitionLabel;

    internal TransitionGraph(IReadOnlyList<Region> regions,
        Func<GraphPosition, TransitionDragTarget?>? resolveDragTarget = null)
    {
        this.regions = regions;
        this.resolveDragTarget = resolveDragTarget;
        Focusable = true;
        UpdateChannels();
    }

    internal event Action<TransitionEdit>? EditProposed;
    internal event Action<TransitionEdit?>? PreviewChanged;
    internal event Action<int>? KeyframeSelected;
    /// <summary>A curve was clicked: Region index, channel, and the Transition (segment) index under the pointer.</summary>
    internal event Action<int, string, int>? CurveSelected;
    internal event Action<GraphBoundary>? BoundarySelected;
    internal event Action<int>? RegionSelected;
    internal event Action? InvalidDrag;
    internal IReadOnlyList<string> Channels => channels;
    internal IReadOnlyList<Region> Regions => regions;
    internal IReadOnlyList<Region> DisplayedRegions => previewRegions ?? regions;
    internal int SelectedRegion => activeRegion;
    internal int SelectedKeyframe => selectedKey;
    internal string RegionName(int index) => regionNames?[index] ?? $"Transition {index + 1}";
    private double DomainStart => domainStart ?? (regions.Count == 0 ? 0 : regions[0].Start);
    private double DomainEnd => domainEnd ?? (regions.Count == 0 ? 1 : regions[^1].End);

    internal TransitionGraphViewport Viewport
    {
        get
        {
            double inset = System.Math.Clamp(Bounds.Width * .042, 22, 56);
            var plot = new Rect(inset, 21, System.Math.Max(0, Bounds.Width - 2 * inset),
                System.Math.Max(0, Bounds.Height - 68));
            return new(plot, viewStart ?? DomainStart, viewEnd ?? DomainEnd, minimumValue, maximumValue);
        }
    }

    /// <summary>Shows a fixed distance domain, such as a whole operational segment, instead of the Regions' extent.</summary>
    internal void SetDomain(double? start, double? end)
    {
        if (start == domainStart && end == domainEnd) { return; }
        domainStart = start;
        domainEnd = end;
        Fit();
    }

    internal IBrush ChannelBrush(string channel) => ChannelPen(channel).Brush!;

    private ImmutablePen ChannelPen(string channel)
    {
        int exampleColor = Array.IndexOf(ExampleChannels, channel);
        int index = exampleColor >= 0 ? exampleColor : channelColors[channel];
        return CurvePens[index % CurvePens.Length];
    }

    internal void SetChannelVisible(string channel, bool visible)
    {
        if (visible)
        {
            hiddenChannels.Remove(channel);
        }
        else
        {
            hiddenChannels.Add(channel);
        }
        InvalidateVisual();
    }

    /// <summary>Highlights one Transition of the editable profile and labels it with its function.</summary>
    internal void SelectTransition(int index, string? label = null)
    {
        selectedTransition = index;
        selectedTransitionLabel = index >= 0 ? label : null;
        InvalidateVisual();
    }

    internal string? SelectedTransitionLabel => selectedTransition >= 0 ? selectedTransitionLabel : null;

    internal bool IsChannelVisible(string channel) => !hiddenChannels.Contains(channel);

    internal void SetRegions(IReadOnlyList<Region> updated, double low, double high, IReadOnlyList<string>? names = null)
    {
        TransitionGraphViewport oldView = Viewport;
        bool extentChanged = regions.Count == 0 || updated.Count == 0 ||
            regions[0].Start != updated[0].Start || regions[^1].End != updated[^1].End;
        CancelDrag();
        regions = updated;
        regionNames = names;
        minimumValue = low;
        maximumValue = high;
        UpdateChannels();
        if (regions.Count == 0)
        {
            Fit();
        }
        else if (extentChanged && viewStart.HasValue)
        {
            double oldSpan = oldView.End - oldView.Start;
            SetView(oldView.Start + oldSpan / 2, oldSpan);
        }
        InvalidateVisual();
    }

    internal void SetRegionPreview(IReadOnlyList<Region>? preview)
    {
        previewRegions = preview;
        InvalidateVisual();
    }

    private void UpdateChannels()
    {
        channels = regions.SelectMany(region => region.Channels.Keys).Distinct(StringComparer.Ordinal).ToArray();
        foreach (string channel in channels)
        {
            channelColors.TryAdd(channel, channelColors.Count);
        }
    }

    internal void SetSelection(int region, string? channel, IReadOnlyList<TimelineKeyframe> keyframes, int selected)
    {
        if (region != activeRegion || channel != editChannel)
        {
            CancelDrag();
        }
        activeRegion = region;
        editChannel = channel;
        keys = keyframes;
        selectedKey = selected;
        InvalidateVisual();
    }

    internal void Zoom(double factor)
    {
        if (regions.Count == 0)
        {
            return;
        }

        double start = Viewport.Start;
        double end = Viewport.End;
        SetView(start + (end - start) / 2, (end - start) * factor);
    }

    internal void Pan(double fraction)
    {
        if (regions.Count == 0)
        {
            return;
        }

        double start = Viewport.Start;
        double span = Viewport.End - start;
        SetView(start + span * (.5 + fraction), span);
    }

    internal void Fit()
    {
        viewStart = null;
        viewEnd = null;
        InvalidateVisual();
    }

    private void SetView(double center, double requestedSpan)
    {
        double start = DomainStart;
        double end = DomainEnd;
        double fullSpan = end - start;
        // Limit screen magnification to a finite window without changing any authored station.
        double span = System.Math.Clamp(requestedSpan, fullSpan / 128, fullSpan);
        double boundedStart = System.Math.Clamp(center - span / 2, start, System.Math.Max(start, end - span));
        double boundedEnd = System.Math.Min(boundedStart + span, end);
        if (boundedEnd <= boundedStart || span == fullSpan)
        {
            Fit();
            return;
        }
        viewStart = boundedStart;
        viewEnd = boundedEnd;
        InvalidateVisual();
    }

    internal IEnumerable<Point[]> CurvePaths(string channel) => CurvePaths(channel, Viewport);

    private IEnumerable<Point[]> CurvePaths(string channel, TransitionGraphViewport viewport) => viewport.HasPlot
        ? DisplayedRegions.Where(region => region.Channels.ContainsKey(channel))
            .SelectMany(region => viewport.Sample(region.Channels[channel]))
        : [];

    public override void Render(DrawingContext context)
    {
        context.FillRectangle(Brushes.Black, new Rect(Bounds.Size));
        TransitionGraphViewport viewport = Viewport;
        if (!viewport.HasPlot)
        {
            return;
        }

        DrawGrid(context, viewport.Plot);
        if (regions.Count == 0)
        {
            // A fixed domain, such as an operational segment, still shows its distance scale.
            if (domainStart.HasValue && domainEnd.HasValue) { DrawRuler(context, viewport); }
            return;
        }

        using (context.PushClip(viewport.Plot.Inflate(2)))
        {
            foreach (string channel in Channels.Where(channel => !hiddenChannels.Contains(channel)))
            {
                var geometry = new StreamGeometry();
                using (StreamGeometryContext path = geometry.Open())
                {
                    foreach (Point[] span in CurvePaths(channel, viewport))
                    {
                        path.BeginFigure(span[0], isFilled: false);
                        for (int i = 1; i < span.Length; i++)
                        {
                            path.LineTo(span[i]);
                        }
                        path.EndFigure(isClosed: false);
                    }
                }
                context.DrawGeometry(null, ChannelPen(channel), geometry);
            }
        }

        DrawRegions(context, viewport);
        DrawRuler(context, viewport);
        DrawSelectedTransition(context, viewport);
        DrawKeyframes(context, viewport);
        DrawDragFeedback(context, viewport);
    }

    private void DrawDragFeedback(DrawingContext context, TransitionGraphViewport viewport)
    {
        Rect plot = viewport.Plot;
        GraphBoundary? highlighted = activeBoundary ?? (dragPointer is null ? hoverBoundary : null);
        if (highlighted is { } boundary)
        {
            double station = edit.Candidate is TransitionEdit.Boundary moved ? moved.Proposed : boundary.Station;
            double x = viewport.ProjectDistance(station);
            if (x >= plot.Left - 1 && x <= plot.Right + 1)
            {
                context.DrawLine(SelectionLine, new Point(x, plot.Top), new Point(x, plot.Bottom));
                context.DrawRectangle(Highlight, null, new Rect(x - 4, plot.Top - 4, 8, 8), 1, 1);
                if (IsDragging)
                {
                    DrawTag(context, $"{station.ToString("G6", CultureInfo.CurrentCulture)} m", x, plot.Top + 4, plot);
                }
            }
        }
        if (IsDragging && edit.Candidate is TransitionEdit.Keyframe key)
        {
            Point point = viewport.Project(new(key.Proposed.Coordinate, key.Proposed.Value));
            string text = $"{key.Proposed.Coordinate.ToString("G6", CultureInfo.CurrentCulture)} m · " +
                key.Proposed.Value.ToString("G6", CultureInfo.CurrentCulture);
            DrawTag(context, text, point.X, System.Math.Clamp(point.Y - 22, plot.Top, plot.Bottom - 14), plot);
        }
    }

    private static void DrawTag(DrawingContext context, string text, double x, double y, Rect plot)
    {
        var label = new FormattedText(text, CultureInfo.CurrentCulture, FlowDirection.LeftToRight, GraphType, 11, Brushes.White);
        double left = System.Math.Clamp(x + 6, plot.Left, System.Math.Max(plot.Left, plot.Right - label.Width - 4));
        context.FillRectangle(LabelBackground, new Rect(left - 2, y - 1, label.Width + 4, label.Height + 2));
        context.DrawText(label, new Point(left, y));
    }

    private void DrawSelectedTransition(DrawingContext context, TransitionGraphViewport viewport)
    {
        if (activeRegion < 0 || activeRegion >= DisplayedRegions.Count ||
            selectedTransition < 0 || selectedTransition + 1 >= keys.Count ||
            editChannel is not { } channel || hiddenChannels.Contains(channel)) { return; }

        double start = System.Math.Max(keys[selectedTransition].Coordinate, viewport.Start);
        double end = System.Math.Min(keys[selectedTransition + 1].Coordinate, viewport.End);
        if (end <= start) { return; }

        double left = viewport.ProjectDistance(start);
        double right = viewport.ProjectDistance(end);
        Rect plot = viewport.Plot;
        var extent = new Rect(left, plot.Top, right - left, plot.Height);
        using (context.PushClip(extent.Inflate(.5)))
        {
            foreach (Point[] path in viewport.Sample(DisplayedRegions[activeRegion].Channels[channel]))
            {
                for (int i = 1; i < path.Length; i++) { context.DrawLine(SelectedCurve, path[i - 1], path[i]); }
            }
        }
        context.DrawLine(ChannelPen(channel), extent.BottomLeft, extent.BottomRight);
        if (selectedTransitionLabel is { } label && !IsDragging)
        {
            // Name the committed function on the selected span so the active choice is visible on the graph.
            DrawTag(context, label, left - 6, plot.Top + 4, plot);
        }
    }

    private void DrawKeyframes(DrawingContext context, TransitionGraphViewport viewport)
    {
        // During a boundary drag the committed keyframe stations no longer match the previewed curves.
        if (editChannel is null || hiddenChannels.Contains(editChannel) ||
            (IsDragging && edit.Candidate is TransitionEdit.Boundary))
        {
            return;
        }

        using var clip = context.PushClip(viewport.Plot.Inflate(2));
        for (int index = 0; index < keys.Count; index++)
        {
            TimelineKeyframe key = keys[index];
            if (index == selectedKey && edit.Candidate is TransitionEdit.Keyframe candidate)
            {
                key = candidate.Proposed;
            }
            Point point = viewport.Project(new(key.Coordinate, key.Value));
            if (!double.IsFinite(point.X) || !double.IsFinite(point.Y))
            {
                continue;
            }
            double radius = index == selectedKey ? 5.5 : 4;
            context.DrawEllipse(index == selectedKey ? Brushes.White : ChannelBrush(editChannel),
                OuterBoundary, point, radius, radius);
        }
    }

    private void DrawGrid(DrawingContext context, Rect plot)
    {
        const double spacing = 16;
        for (int column = 0; column * spacing < Bounds.Width; column++)
        {
            for (int row = 0; plot.Top + row * spacing <= plot.Bottom; row++)
            {
                bool major = column % 5 == 0 && row % 5 == 0;
                double radius = major ? 1.5 : .85;
                context.DrawEllipse(major ? MajorGridDot : GridDot, null,
                    new Point(column * spacing, plot.Top + row * spacing), radius, radius);
            }
        }
    }

    private void DrawRegions(DrawingContext context, TransitionGraphViewport viewport)
    {
        IReadOnlyList<Region> regions = DisplayedRegions;
        for (int i = 0; i < regions.Count; i++)
        {
            double left = viewport.ProjectDistance(regions[i].Start);
            double right = viewport.ProjectDistance(regions[i].End);
            double visibleLeft = System.Math.Max(left, viewport.Plot.Left);
            double visibleRight = System.Math.Min(right, viewport.Plot.Right);
            DrawCenteredText(context, RegionName(i), (visibleLeft + visibleRight) / 2, 2,
                visibleRight - visibleLeft - 6, i == activeRegion ? Brushes.White : Label);
            if (i == activeRegion && visibleRight > visibleLeft)
            {
                context.DrawLine(SelectionLine, new Point(visibleLeft, 15), new Point(visibleRight, 15));
            }
            bool sharedStart = i > 0 && regions[i - 1].End == regions[i].Start;
            if (regions[i].Start >= viewport.Start && regions[i].Start <= viewport.End)
            {
                DrawBoundary(context, viewport.Plot, left, isInterior: sharedStart,
                    selected: i == activeRegion || (sharedStart && i - 1 == activeRegion));
            }
            bool sharedEnd = i + 1 < regions.Count && regions[i].End == regions[i + 1].Start;
            if (!sharedEnd && regions[i].End >= viewport.Start && regions[i].End <= viewport.End)
            {
                DrawBoundary(context, viewport.Plot, right, isInterior: false, selected: i == activeRegion);
            }
        }
    }

    private static void DrawBoundary(DrawingContext context, Rect plot, double x, bool isInterior, bool selected)
    {
        context.DrawLine(isInterior ? InnerBoundary : OuterBoundary, new Point(x, plot.Top), new Point(x, plot.Bottom));
        // The selected Region's boundary handles use the selection color shown under its title.
        context.DrawRectangle(selected ? Highlight : Brushes.White, null, new Rect(x - 3.5, plot.Top - 3.5, 7, 7), 1, 1);
        if (isInterior)
        {
            // These shapes follow the reference artwork. Target selection is supplied independently.
            double center = plot.Bottom + 8;
            var diamond = new StreamGeometry();
            using (StreamGeometryContext shape = diamond.Open())
            {
                shape.BeginFigure(new Point(x, center - 5), isFilled: true);
                shape.LineTo(new Point(x + 3.5, center));
                shape.LineTo(new Point(x, center + 5));
                shape.LineTo(new Point(x - 3.5, center));
                shape.EndFigure(isClosed: true);
            }
            context.DrawGeometry(Brushes.White, null, diamond);
        }
    }

    private static void DrawRuler(DrawingContext context, TransitionGraphViewport viewport)
    {
        Rect plot = viewport.Plot;
        double y = plot.Bottom + 16;
        context.DrawLine(Ruler, new Point(plot.Left, y), new Point(plot.Right, y));
        DrawCenteredText(context, "m", 9, y + 17);

        // Choose a labelled interval with space for its text at the current magnification.
        double interval = (viewport.End - viewport.Start) / System.Math.Max(2, plot.Width / 56);
        double decade = System.Math.Pow(10, System.Math.Floor(System.Math.Log10(interval)));
        double majorStep = decade * (interval <= decade ? 1 : interval <= 2 * decade ? 2 : interval <= 5 * decade ? 5 : 10);
        double tickStep = majorStep / 5;
        double firstTick = System.Math.Ceiling(viewport.Start / tickStep);
        if (!(tickStep > 0) || !double.IsFinite(majorStep) || firstTick >= 9_007_199_254_740_992)
        {
            DrawCenteredText(context, viewport.Start.ToString("R", CultureInfo.InvariantCulture), plot.Left, y + 17);
            DrawCenteredText(context, viewport.End.ToString("R", CultureInfo.InvariantCulture), plot.Right, y + 17);
            return;
        }
        int ticks = (int)System.Math.Floor(viewport.End / tickStep - firstTick);
        for (int tick = 0; tick <= ticks; tick++)
        {
            double tickIndex = firstTick + tick;
            bool major = tickIndex % 5 == 0;
            double station = tickIndex * tickStep;
            double x = viewport.ProjectDistance(station);
            context.DrawLine(Ruler, new Point(x, y), new Point(x, y + (major ? 9 : 5)));
            if (major)
            {
                DrawCenteredText(context, station.ToString("G6", CultureInfo.InvariantCulture), x, y + 17);
            }
        }
    }

    private static void DrawCenteredText(DrawingContext context, string text, double x, double y,
        double availableWidth = double.PositiveInfinity, IBrush? color = null)
    {
        var label = new FormattedText(text, CultureInfo.InvariantCulture, FlowDirection.LeftToRight, GraphType, 11, color ?? Label);
        if (label.Width <= availableWidth)
        {
            context.DrawText(label, new Point(x - label.Width / 2, y));
        }
    }

    private static ImmutablePen CurvePen(string color) => new(new ImmutableSolidColorBrush(Color.Parse(color)), 2.2,
        lineCap: PenLineCap.Round, lineJoin: PenLineJoin.Round);
}
