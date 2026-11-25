using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;

namespace QuantumDynamix.App;

internal static class ProjectFile
{
    internal const string Extension = "qdproject";
    private const string Format = "quantum-dynamix-project";
    private const int Version = 2;
    private const string Linear = "linear";
    private const string Piecewise = "piecewise";

    private static readonly JsonSerializerOptions Json = new()
    {
        PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
        WriteIndented = true,
        RespectNullableAnnotations = true,
        RespectRequiredConstructorParameters = true,
        UnmappedMemberHandling = JsonUnmappedMemberHandling.Disallow
    };

    private sealed record ProjectData(string Format, int Version, TrackData Track);
    private sealed record TrackData(string Name, double ReconstructionSpeedMetresPerSecond,
        FrameData StartingFrame, AnchorData? ReconstructionAnchor, RegionData[] Regions);
    private sealed record AnchorData(double StationMetres, FrameData Frame,
        [property: JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)] double? GeometricHeadingRadians = null);
    private sealed record RegionData(Guid Id, string Name, double StartMetres, double EndMetres, string Mode,
        ProfileData[] Profiles);
    private sealed record ProfileData(string Channel, string Interpolation, KeyData[] Keyframes,
        [property: JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)] TransitionData[]? Transitions = null);
    private sealed record TransitionData(string Type,
        [property: JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)] string? Direction = null,
        [property: JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)] PlateauData? Plateau = null);
    private sealed record PlateauData(double HoldStart, double HoldEnd, double HoldValue);
    private sealed record KeyData(double StationMetres, double Value);

    private sealed record VectorData(double X, double Y, double Z)
    {
        internal static VectorData From(Vector3d vector) => new(vector.X, vector.Y, vector.Z);
        internal Vector3d ToVector() => new(X, Y, Z);
    }

    private sealed record FrameData(VectorData PositionMetres, VectorData Forward, VectorData Left, VectorData Up)
    {
        internal static FrameData From(CurveFrame frame) =>
            new(VectorData.From(frame.Position), VectorData.From(frame.Forward),
                VectorData.From(frame.Left), VectorData.From(frame.Up));

        internal CurveFrame ToFrame(string label)
        {
            if (!IsAxis(Forward) || !IsAxis(Left) || !IsAxis(Up))
            {
                throw new InvalidDataException($"The {label} must have finite unit axes.");
            }
            Vector3d forward = Forward.ToVector();
            Vector3d left = Left.ToVector();
            Vector3d up = Up.ToVector();
            // Check the representation on disk while keeping the exact serialized axes used by reconstruction.
            if ((Vector3d.Cross(forward, left) - up).Length > 1e-12)
            {
                throw new InvalidDataException($"The {label} must be a right-handed, orthonormal frame.");
            }
            return new(PositionMetres.ToVector(), forward, left, up);
        }

        private static bool IsAxis(VectorData axis) =>
            double.IsFinite(axis.X) && double.IsFinite(axis.Y) && double.IsFinite(axis.Z) &&
            System.Math.Abs(axis.ToVector().Length - 1) <= 1e-12;
    }

    internal static void Save(string path, TrackDocument document)
    {
        TimelineEditorSession session = document.Timeline;
        if (session.ReconstructionSpeed is not double speed)
        {
            throw new InvalidOperationException("Only authored tracks can be saved as projects.");
        }

        AnchorData? anchor = session.ReconstructionAnchor is { } origin
            ? new(origin.Station, FrameData.From(origin.Frame), origin.GeometricHeadingRadians) : null;
        var track = new TrackData(document.Identity.Name, speed, FrameData.From(session.InitialFrame), anchor,
            session.Regions.Select(WriteRegion).ToArray());
        int version = anchor?.GeometricHeadingRadians is not null || track.Regions.Any(region => region.Mode == "geometric" ||
            region.Profiles.Any(profile => profile.Transitions is not null)) ? Version : 1;
        string target = Path.GetFullPath(path);
        string temporary = Path.Combine(Path.GetDirectoryName(target)!, $".{Guid.NewGuid():N}.tmp");
        bool ownsTemporary = false;
        try
        {
            using (var output = new FileStream(temporary, FileMode.CreateNew, FileAccess.Write, FileShare.None))
            {
                ownsTemporary = true;
                JsonSerializer.Serialize(output, new ProjectData(Format, version, track), Json);
                output.Flush(flushToDisk: true);
            }
            File.Move(temporary, target, overwrite: true);
            ownsTemporary = false;
        }
        finally
        {
            if (ownsTemporary)
            {
                try { File.Delete(temporary); }
                catch (Exception error) when (error is IOException or UnauthorizedAccessException)
                {
                    // Keep the original save error if the filesystem also prevents cleanup.
                }
            }
        }
    }

    private static RegionData WriteRegion(AuthoredRegion entry) =>
        new(entry.Id, entry.Name, entry.Definition.Start, entry.Definition.End, entry.Mode switch
        {
            RegionAuthoringMode.CoordinateAcceleration => "coordinateAcceleration",
            RegionAuthoringMode.Force => "force",
            RegionAuthoringMode.Geometric => "geometric",
            _ => throw new InvalidDataException($"Unsupported authoring mode in '{entry.Name}'.")
        }, entry.Keyframes.Select(pair =>
        {
            TransitionShape[] shapes;
            try { shapes = entry.Shapes(pair.Key); }
            catch (InvalidOperationException error)
            {
                throw new InvalidDataException($"Unsupported interpolation in '{entry.Name}', channel '{pair.Key}'.", error);
            }
            bool linear = shapes.All(shape => shape == TransitionShape.Linear);
            return new ProfileData(pair.Key, linear ? Linear : Piecewise,
                pair.Value.Select(key => new KeyData(key.Coordinate, key.Value)).ToArray(),
                linear ? null : shapes.Select(WriteTransition).ToArray());
        }).ToArray());

    private static TransitionData WriteTransition(TransitionShape shape) => new(
        JsonNamingPolicy.CamelCase.ConvertName(shape.Name),
        shape.Kind is null ? null : JsonNamingPolicy.CamelCase.ConvertName(shape.Direction.ToString()),
        shape.Kind == EasingKind.Plateau ? new(shape.HoldStart, shape.HoldEnd, shape.HoldValue) : null);

    internal static TrackDocument Load(string path)
    {
        using var input = File.OpenRead(path);
        ReadOnlySpan<byte> utf8Bom = [0xEF, 0xBB, 0xBF];
        Span<byte> prefix = stackalloc byte[3];
        if (input.ReadAtLeast(prefix, prefix.Length, false) != prefix.Length || !prefix.SequenceEqual(utf8Bom))
        {
            input.Position = 0;
        }

        try
        {
            using var json = JsonDocument.Parse(input, new JsonDocumentOptions { AllowDuplicateProperties = false });
            JsonElement root = json.RootElement;
            if (root.ValueKind != JsonValueKind.Object || !root.TryGetProperty("format", out JsonElement format) ||
                format.ValueKind != JsonValueKind.String || format.GetString() != Format)
            {
                throw new InvalidDataException("This file is not a QUANTUM DYNAMIX project.");
            }
            if (!root.TryGetProperty("version", out JsonElement version) || version.ValueKind != JsonValueKind.Number ||
                !version.TryGetInt32(out int number))
            {
                throw new InvalidDataException("The project needs an integer format version.");
            }
            if (number is < 1 or > Version)
            {
                throw new InvalidDataException($"Project format version {number} is unsupported. This application reads versions 1 and {Version}.");
            }

            return ReadTrack(root.Deserialize<ProjectData>(Json)!.Track, number);
        }
        catch (JsonException error)
        {
            string location = error.Path is { Length: > 0 } source ? $" at {source}"
                : $" near line {error.LineNumber.GetValueOrDefault() + 1}";
            throw new InvalidDataException($"Invalid or incomplete project JSON{location}.", error);
        }
        catch (Exception error) when (error is ArgumentException or InvalidOperationException or ArithmeticException)
        {
            throw new InvalidDataException(error.Message, error);
        }
    }

    private static TrackDocument ReadTrack(TrackData track, int version)
    {
        if (string.IsNullOrWhiteSpace(track.Name))
        {
            throw new InvalidDataException("The project needs a track name.");
        }
        if (!RegionTrackGeometry.SupportsSpeed(track.ReconstructionSpeedMetresPerSecond))
        {
            throw new InvalidDataException("The project needs a finite reconstruction speed of at least 0.0001 m/s.");
        }

        var ids = new HashSet<Guid>();
        var regions = new AuthoredRegion[track.Regions.Length];
        for (int i = 0; i < regions.Length; i++)
        {
            RegionData source = track.Regions[i] ?? throw new InvalidDataException($"Region {i + 1} is missing.");
            if (source.Id == Guid.Empty || !ids.Add(source.Id))
            {
                throw new InvalidDataException($"Region {i + 1} needs a unique, nonempty ID.");
            }
            if (i > 0 && regions[i - 1].Definition.End != source.StartMetres)
            {
                throw new InvalidDataException($"Region {i + 1} must start at the preceding Region's end station.");
            }
            regions[i] = ReadRegion(source, i + 1, version);
        }

        TrackReconstructionAnchor? anchor = null;
        if (regions.Length > 0)
        {
            AnchorData origin = track.ReconstructionAnchor
                ?? throw new InvalidDataException("A track with Regions needs a reconstruction anchor.");
            Region first = regions[0].Definition;
            if (origin.StationMetres != first.Start && (regions.Length < 2 || origin.StationMetres != first.End))
            {
                throw new InvalidDataException("The reconstruction anchor must be at the track start or the first Region join.");
            }
            if (origin.GeometricHeadingRadians is double heading &&
                (version < 2 || !double.IsFinite(heading) || System.Math.Abs(heading) > System.Math.PI))
            {
                throw new InvalidDataException("The geometric anchor heading requires version 2 and an angle from −π to π.");
            }
            anchor = new(origin.StationMetres, origin.Frame.ToFrame("reconstruction anchor"), origin.GeometricHeadingRadians);
        }
        else if (track.ReconstructionAnchor is not null)
        {
            throw new InvalidDataException("A track without Regions cannot have a reconstruction anchor.");
        }

        return new(track.Name, timeline: TimelineEditorSession.RestoreAuthored(
            track.StartingFrame.ToFrame("starting frame"), track.ReconstructionSpeedMetresPerSecond, regions, anchor));
    }

    private static AuthoredRegion ReadRegion(RegionData source, int number, int version)
    {
        string label = $"Region {number}";
        if (string.IsNullOrWhiteSpace(source.Name))
        {
            throw new InvalidDataException($"{label} needs a name.");
        }
        if (!double.IsFinite(source.StartMetres) || !double.IsFinite(source.EndMetres) ||
            source.StartMetres < 0 || source.EndMetres <= source.StartMetres)
        {
            throw new InvalidDataException($"{label} needs a finite, positive heartline interval starting at or after 0 m.");
        }
        RegionAuthoringMode mode = source.Mode switch
        {
            "coordinateAcceleration" => RegionAuthoringMode.CoordinateAcceleration,
            "force" => RegionAuthoringMode.Force,
            "geometric" when version >= 2 => RegionAuthoringMode.Geometric,
            _ => throw new InvalidDataException($"{label} has an unsupported authoring mode '{source.Mode}'.")
        };
        var (lateral, vertical) = RegionTrackGeometry.TransverseChannels(mode);
        var keys = new Dictionary<string, TimelineKeyframe[]>(StringComparer.Ordinal);
        var channels = new Dictionary<string, ScalarTimeline>(StringComparer.Ordinal);
        foreach (ProfileData? profile in source.Profiles)
        {
            if (profile is null || string.IsNullOrWhiteSpace(profile.Channel) || keys.ContainsKey(profile.Channel))
            {
                throw new InvalidDataException($"{label} has a missing, blank, or duplicate channel.");
            }
            if (RegionTrackGeometry.IsTransverseChannel(profile.Channel) && profile.Channel != lateral && profile.Channel != vertical)
            {
                throw new InvalidDataException($"{label} has a transverse profile that does not match its authoring mode.");
            }
            TimelineKeyframe[] frames = profile.Keyframes.Select(key => key is null
                ? throw new InvalidDataException($"{label}, channel '{profile.Channel}', has a missing keyframe.")
                : new TimelineKeyframe(key.StationMetres, key.Value)).ToArray();
            if (frames.Length < 2 || frames[0].Coordinate != source.StartMetres || frames[^1].Coordinate != source.EndMetres)
            {
                throw new InvalidDataException($"{label}, channel '{profile.Channel}', needs keyframes at both Region endpoints.");
            }
            TransitionShape[]? shapes = profile switch
            {
                { Interpolation: Linear, Transitions: null } => null,
                { Interpolation: Piecewise, Transitions: { } transitions } when version >= 2 =>
                    transitions.Select(transition => ReadTransition(transition, $"{label}, channel '{profile.Channel}'")).ToArray(),
                _ => throw new InvalidDataException($"{label}, channel '{profile.Channel}', uses unsupported interpolation '{profile.Interpolation}' or has unexpected or missing transition data.")
            };
            if (TimelineEditorSession.BuildTimeline(ProfileDomain.Distance, frames, out ScalarTimeline? timeline, shapes) is { } error)
            {
                throw new InvalidDataException($"{label}, channel '{profile.Channel}'. {error}");
            }
            keys.Add(profile.Channel, frames);
            channels.Add(profile.Channel, timeline!);
        }
        if (!keys.ContainsKey(TrackDocument.BankingChannel) || !keys.ContainsKey(lateral) || !keys.ContainsKey(vertical))
        {
            throw new InvalidDataException($"{label} needs banking and both transverse profiles for its authoring mode.");
        }
        return new(source.Id, source.Name, new Region(ProfileDomain.Distance, source.StartMetres, source.EndMetres, channels),
            keys, mode);
    }

    private static TransitionShape ReadTransition(TransitionData? source, string label)
    {
        if (source is { Type: "constant", Direction: null, Plateau: null }) { return new(null); }
        if (source is null || !TryReadEnum(source.Type, out EasingKind kind) || kind == EasingKind.Custom ||
            !TryReadEnum(source.Direction, out EasingDirection direction) ||
            (kind == EasingKind.Plateau) != (source.Plateau is not null))
        {
            throw new InvalidDataException($"{label} has an unsupported transition type, direction, or parameter set.");
        }
        return source.Plateau is { } plateau ? new(kind, direction, plateau.HoldStart, plateau.HoldEnd, plateau.HoldValue)
            : new(kind, direction);
    }

    private static bool TryReadEnum<T>(string? text, out T value) where T : struct, Enum =>
        Enum.TryParse(text, ignoreCase: true, out value) && Enum.IsDefined(value) &&
        JsonNamingPolicy.CamelCase.ConvertName(value.ToString()) == text;
}
