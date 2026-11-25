using Avalonia;
using Avalonia.Controls;
using Avalonia.Layout;
using Avalonia.Media;

namespace QuantumDynamix.App;

internal enum SaveChangesChoice { Cancel, Save, Discard }

internal sealed class SaveChangesDialog : Window
{
    internal SaveChangesDialog(string projectName)
    {
        Title = "Unsaved Changes";
        Width = 390;
        SizeToContent = SizeToContent.Height;
        CanResize = false;
        WindowStartupLocation = WindowStartupLocation.CenterOwner;
        Background = Brushes.Black;
        Foreground = Brushes.White;
        FontFamily = new("Cascadia Mono, Consolas, DejaVu Sans Mono, monospace");

        var buttons = new StackPanel
        {
            Orientation = Orientation.Horizontal, HorizontalAlignment = HorizontalAlignment.Right, Spacing = 8
        };
        var cancel = new Button { Content = "Cancel", IsCancel = true };
        var discard = new Button { Content = "Discard" };
        var save = new Button { Content = "Save", IsDefault = true };
        cancel.Click += (_, _) => Close(SaveChangesChoice.Cancel);
        discard.Click += (_, _) => Close(SaveChangesChoice.Discard);
        save.Click += (_, _) => Close(SaveChangesChoice.Save);
        buttons.Children.Add(cancel);
        buttons.Children.Add(discard);
        buttons.Children.Add(save);
        Content = new StackPanel
        {
            Spacing = 12,
            Margin = new Thickness(16),
            Children = { new TextBlock
            {
                Text = $"Save changes to “{projectName}” before continuing?",
                TextWrapping = TextWrapping.Wrap
            }, buttons }
        };
    }
}
