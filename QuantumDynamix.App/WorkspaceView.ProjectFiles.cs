using System;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using Avalonia.Controls;
using Avalonia.Interactivity;
using Avalonia.Platform.Storage;

namespace QuantumDynamix.App;

internal sealed partial class WorkspaceView
{
    private static readonly FilePickerFileType NativeProjectType = new("QUANTUM DYNAMIX project")
    {
        Patterns = [$"*.{ProjectFile.Extension}"]
    };

    private string? projectPath;
    private Window OwnerWindow => (Window)TopLevel.GetTopLevel(this)!;
    private bool HasAuthoredTrack => track?.Timeline.SupportsForceRegions == true;
    internal bool HasUnsavedChanges => HasAuthoredTrack && track!.Timeline.HasUnsavedChanges;
    internal bool ProjectOperationInProgress { get; private set; }
    internal Func<FilePickerOpenOptions, Task<string?>>? ProjectOpenPicker { private get; set; }
    internal Func<FilePickerSaveOptions, Task<string?>>? ProjectSavePicker { private get; set; }

    private void RefreshProjectControls()
    {
        SaveProjectMenu.IsEnabled = SaveProjectAsMenu.IsEnabled = HasAuthoredTrack;
        if (TopLevel.GetTopLevel(this) is MainWindow window)
        {
            window.Title = HasAuthoredTrack
                ? $"{track!.Identity.Name}{(HasUnsavedChanges ? " *" : "")} — {MainWindow.AppTitle}"
                : MainWindow.AppTitle;
        }
    }

    private async void OpenProject(object? sender, RoutedEventArgs e) =>
        await RunProjectOperation(OpenProjectFile);

    private async void SaveProject(object? sender, RoutedEventArgs e) =>
        await RunProjectOperation(async () => { await SaveCurrentProject(saveAs: false); });

    private async void SaveProjectAs(object? sender, RoutedEventArgs e) =>
        await RunProjectOperation(async () => { await SaveCurrentProject(saveAs: true); });

    private async Task OpenProjectFile()
    {
        try
        {
            string? path = await PickProjectToOpen();
            if (path is null || !await ConfirmReplaceProject()) { return; }
            TrackDocument opened = await Task.Run(() => ProjectFile.Load(path));
            OpenTrack(opened, path);
            StatusLabel.Text = $"Opened {Path.GetFileName(path)}.";
        }
        catch (Exception error) when (IsProjectFileError(error))
        {
            StatusLabel.Text = $"Could not open project. {error.Message}";
        }
    }

    private async Task<bool> SaveCurrentProject(bool saveAs)
    {
        if (!HasAuthoredTrack) { return false; }
        TrackDocument document = track!;
        try
        {
            string? path = saveAs || projectPath is null ? await PickProjectToSave(document) : projectPath;
            if (path is null) { return false; }
            await Task.Run(() => ProjectFile.Save(path, document));
            projectPath = path;
            document.Timeline.MarkSaved();
            RefreshProjectControls();
            StatusLabel.Text = $"Saved {Path.GetFileName(path)}.";
            return true;
        }
        catch (Exception error) when (IsProjectFileError(error))
        {
            StatusLabel.Text = $"Could not save project. {error.Message}";
            return false;
        }
    }

    private async Task<string?> PickProjectToOpen()
    {
        var options = new FilePickerOpenOptions
        {
            Title = "Open Project",
            FileTypeFilter = [NativeProjectType],
            AllowMultiple = false
        };
        if (ProjectOpenPicker is { } pick) { return await pick(options); }
        IStorageProvider storage = OwnerWindow.StorageProvider;
        if (!storage.CanOpen) { throw new IOException("The file picker is unavailable."); }
        using IStorageFile? file = (await storage.OpenFilePickerAsync(options)).FirstOrDefault();
        return LocalPath(file);
    }

    private async Task<string?> PickProjectToSave(TrackDocument document)
    {
        char[] invalid = Path.GetInvalidFileNameChars();
        string suggestedName = new(document.Identity.Name.Select(c => invalid.Contains(c) ? '_' : c).ToArray());
        var options = new FilePickerSaveOptions
        {
            Title = "Save Project As",
            SuggestedFileName = projectPath is null ? suggestedName : Path.GetFileNameWithoutExtension(projectPath),
            DefaultExtension = ProjectFile.Extension,
            FileTypeChoices = [NativeProjectType],
            ShowOverwritePrompt = true
        };
        if (ProjectSavePicker is { } pick) { return await pick(options); }
        IStorageProvider storage = OwnerWindow.StorageProvider;
        if (!storage.CanSave) { throw new IOException("The file picker is unavailable."); }
        using IStorageFolder? folder = projectPath is null ? null
            : await storage.TryGetFolderFromPathAsync(Path.GetDirectoryName(projectPath)!);
        options.SuggestedStartLocation = folder;
        using IStorageFile? file = await storage.SaveFilePickerAsync(options);
        return LocalPath(file);
    }

    private static string? LocalPath(IStorageFile? file) => file is null ? null
        : file.TryGetLocalPath() ?? throw new IOException("Choose a project file on the local filesystem.");

    private static bool IsProjectFileError(Exception error) =>
        error is IOException or UnauthorizedAccessException or InvalidDataException or ArgumentException
            or InvalidOperationException or NotSupportedException;

    private async Task RunProjectOperation(Func<Task> operation)
    {
        if (ProjectOperationInProgress) { return; }
        ProjectOperationInProgress = true;
        IsEnabled = false;
        try { await operation(); }
        finally
        {
            IsEnabled = true;
            ProjectOperationInProgress = false;
        }
    }

    internal async Task<bool> ConfirmCloseProject()
    {
        bool close = false;
        await RunProjectOperation(async () => { close = await ConfirmReplaceProject(); });
        return close;
    }

    private async Task<bool> ConfirmReplaceProject()
    {
        if (!HasUnsavedChanges) { return true; }
        var dialog = new SaveChangesDialog(track!.Identity.Name);
        return (await dialog.ShowDialog<SaveChangesChoice>(OwnerWindow)) switch
        {
            SaveChangesChoice.Save => await SaveCurrentProject(saveAs: false),
            SaveChangesChoice.Discard => true,
            _ => false
        };
    }
}
