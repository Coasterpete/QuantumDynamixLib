using System.Runtime.CompilerServices;

// Numerical derivative regressions inspect internal row assembly without exposing a public matrix API.
[assembly: InternalsVisibleTo("QuantumDynamix.Tests")]
