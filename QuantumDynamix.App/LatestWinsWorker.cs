using System;
using System.Threading.Tasks;

namespace QuantumDynamix.App;

/// <summary>Runs one computation at a time off the UI thread and applies only results that are still current.</summary>
/// <remarks>
/// A request made while a computation is running replaces any older waiting request, so a fast drag computes
/// only the newest pointer position after each result. Use it from one thread with a synchronization context,
/// such as the UI thread. Results are applied on that context.
/// </remarks>
internal sealed class LatestWinsWorker<TResult>(Action<TResult> apply)
{
    private int generation;
    private Func<TResult>? pending;
    private bool running;
    private TaskCompletionSource? idle;

    internal bool IsBusy => running;

    internal void Request(Func<TResult> compute)
    {
        pending = compute;
        if (!running)
        {
            _ = RunAsync();
        }
    }

    /// <summary>Discards waiting and running work. A running computation finishes, but its result is ignored.</summary>
    internal void Cancel()
    {
        generation++;
        pending = null;
    }

    internal Task WhenIdle() => running ? (idle ??= new(TaskCreationOptions.RunContinuationsAsynchronously)).Task : Task.CompletedTask;

    private async Task RunAsync()
    {
        running = true;
        try
        {
            while (pending is { } next)
            {
                pending = null;
                int requested = generation;
                TResult result = await Task.Run(next);
                if (requested == generation)
                {
                    apply(result);
                }
            }
        }
        finally
        {
            running = false;
            idle?.TrySetResult();
            idle = null;
        }
    }
}
