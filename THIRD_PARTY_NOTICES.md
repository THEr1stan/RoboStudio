# Third-party components

RoboStudio's original robot geometry and UI assets are maintained in this repository.
No third-party robot models, paper PDFs, weights, or showcase videos are bundled in source exports.

| Component | Pinned version | License / upstream |
|---|---|---|
| CPython (desktop bundle) | Build machine's CPython 3.12 patch release; recorded in runtime-manifest.json | [PSF license](https://docs.python.org/3/license.html); runtime/LICENSE.txt |
| MuJoCo Core | 4.0.1 | [MuJoCo](https://github.com/google-deepmind/mujoco), Apache-2.0; installed distribution license files retained |
| NumPy | 2.5.4 on Python 3.12+ | [NumPy](https://numpy.org/doc/stable/license.html), BSD-3-Clause; NumPy/OpenBLAS notices retained in dist-info |
| Microsoft WebView2 SDK | 1.0.4314.35 | [Microsoft package](https://www.nuget.org/packages/Microsoft.Web.WebView2/1.0.4314.35), BSD-3-Clause; copied LICENSE and NOTICE in desktop bundle |
| WebView2 Evergreen Runtime | System managed | Microsoft runtime license; separate installed prerequisite, not redistributed in this bundle |

Python and JavaScript developer tools are separate from the desktop distribution.
The original RoboStudio source is licensed under MIT; see the root LICENSE.
Third-party components retain their own licenses. The MIT license does not relicense
CPython, MuJoCo, NumPy, OpenBLAS or Microsoft components. Keep their notices when redistributing
the portable app. WebView2 Evergreen Runtime is a separately installed system prerequisite.
