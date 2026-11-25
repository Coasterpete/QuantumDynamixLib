using System;
using System.Collections.Generic;
using Avalonia;
using Avalonia.Automation;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Layout;
using Avalonia.Media;

namespace QuantumDynamix.App;

internal sealed class AuthoringDialog : Window
{
    private readonly Dictionary<string, TextBox> inputs = new(StringComparer.Ordinal);

    internal AuthoringDialog(string title, string action, string? message,
        Func<AuthoringDialog, string?> commit, (string Label, string Initial)[] fields, Control? option = null)
    {
        Title = title;
        Width = 390;
        SizeToContent = SizeToContent.Height;
        CanResize = false;
        WindowStartupLocation = WindowStartupLocation.CenterOwner;
        Background = Brushes.Black;
        Foreground = Brushes.White;
        FontFamily = new("Cascadia Mono, Consolas, DejaVu Sans Mono, monospace");

        var panel = new StackPanel { Spacing = 6, Margin = new Thickness(16) };
        if (message is not null)
        {
            panel.Children.Add(new TextBlock { Text = message, TextWrapping = TextWrapping.Wrap });
        }
        if (option is not null) { panel.Children.Add(option); }
        foreach ((string label, string initial) in fields)
        {
            panel.Children.Add(new TextBlock { Text = label });
            var input = new TextBox { Text = initial };
            AutomationProperties.SetName(input, label);
            inputs.Add(label, input);
            panel.Children.Add(input);
        }

        var error = new TextBlock { TextWrapping = TextWrapping.Wrap, IsVisible = false, Foreground = Brushes.OrangeRed };
        AutomationProperties.SetLiveSetting(error, AutomationLiveSetting.Polite);
        panel.Children.Add(error);
        var accept = new Button { Content = action, IsDefault = true };
        var cancel = new Button { Content = "Cancel", IsCancel = true };
        var buttons = new StackPanel
        {
            Orientation = Orientation.Horizontal, HorizontalAlignment = HorizontalAlignment.Right, Spacing = 8
        };
        buttons.Children.Add(cancel);
        buttons.Children.Add(accept);
        panel.Children.Add(buttons);
        Content = panel;

        accept.Click += (_, _) =>
        {
            string? problem = commit(this);
            if (problem is null)
            {
                Close();
                return;
            }
            error.Text = problem;
            error.IsVisible = true;
        };
        cancel.Click += (_, _) => Close();
        Opened += (_, _) => inputs[fields[0].Label].Focus(NavigationMethod.Tab);
    }

    internal string Read(string label) => inputs[label].Text?.Trim() ?? "";
}
