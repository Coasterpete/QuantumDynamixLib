using System.Threading.Tasks;
using Avalonia.Controls;
using Avalonia.Interactivity;

namespace QuantumDynamix.App;

internal sealed partial class MainWindow : Window
{
    internal const string AppTitle = "QUANTUM DYNAMIX";
    private bool closeApproved;

    public MainWindow()
    {
        InitializeComponent();
    }

    private void OpenEditor(object? sender, RoutedEventArgs e)
    {
        Content = new WorkspaceView();
    }

    private void Exit(object? sender, RoutedEventArgs e)
    {
        Close();
    }

    protected override void OnClosing(WindowClosingEventArgs e)
    {
        base.OnClosing(e);
        if (e.Cancel || closeApproved || Content is not WorkspaceView workspace) { return; }
        if (workspace.ProjectOperationInProgress)
        {
            e.Cancel = true;
        }
        else if (workspace.HasUnsavedChanges)
        {
            e.Cancel = true;
            _ = CloseAfterConfirmation(workspace);
        }
    }

    private async Task CloseAfterConfirmation(WorkspaceView workspace)
    {
        if (!await workspace.ConfirmCloseProject()) { return; }
        closeApproved = true;
        try { Close(); }
        finally { closeApproved = false; }
    }
}
