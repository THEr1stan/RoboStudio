# ADR 0004 — Native Windows host and conservative layout migration

Status: accepted for v0.8.0, at the user's explicit request to reorganize and ship a desktop App.
The user approved Microsoft.Web.WebView2 1.0.4314.35 as a new dependency on 2026-11-17.

The Python/MuJoCo simulation and JavaScript/WebGL view already use stable local APIs.
Use a small C# WinForms/WebView2 host, with bundled CPython 3.12 and existing physics dependencies.
No generic JavaScript-to-native command bridge or host object exposure. Top-level navigation
stays on the owned loopback origin; permission requests are denied.

The host owns a dedicated backend listener and its process tree. Stdin EOF and a Windows job
tie backend lifetime to the window. Readiness is explicit, and port collision cannot attach to
another process. The WebView profile and exports stay in the portable Data directory.
Production has no CDP endpoint; opt-in local debug arguments serve desktop QA.

Move studio implementations into simulation, preserving module identity through legacy aliases.
Move assets into ui/studio and ui/legacy and update the server allowlist, leaving HTTP routes and
API payloads unchanged. Models and numeric algorithms remain unchanged. Core contracts, CLI
commands and old Python imports continue to work.

Developer migration: use robostudio.simulation.* for new imports. Browser tests now live in
tests/browser, validation scripts in scripts/validation, history in docs/history and releases
in docs/releases. Replay helper paths remain stable. Source hashes intentionally change;
old strict replay requires old source; cross-version physical parity is separately tested.

Limitations: Windows x64 portable distribution first; requires installed .NET Framework/WebView2,
is not code-signed, and does not provide an installer or automatic updates. The frontend still
uses WebView technology. Browser localStorage migrates through scene JSON export/import.
