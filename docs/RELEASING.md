# Windows and macOS releases

Users install a prebuilt app; they do not install Rust, Node, ARM GNU, Arduino,
Homebrew, or MSYS2. Windows x64 gets an NSIS setup EXE with the offline WebView2
installer. Macs get separate Apple Silicon and Intel DMGs (macOS 12 or later).
Each app includes the matching Teensy 4.1 HEX, a native `teensy_loader_cli`, force
calibration, and release metadata. ARM GNU and the other compilers run only on CI.

Downloads: <https://github.com/venkatkidambi/teensy-panel-releases/releases>.
The application source remains in the private `venkatkidambi/teensy-usb-scan` repo.

## Stable and nightly channels

`.github/workflows/release.yml` runs for:

- **Pull requests:** validate and build all three installers without release
  signing credentials or publication. Download them from the run's artifacts.

- **Stable:** a pushed `vMAJOR.MINOR.PATCH` tag. The tag must match
  `desktop/tauri.conf.json`. Update the two Cargo package versions and lockfiles
  when bumping the application version, too.
- **Nightly:** pushes to `main`, daily at 06:23 UTC, and manual workflow runs on
  `main`. Versions are `BASE-nightly.RUN_NUMBER.RUN_ATTEMPT`; every published
  nightly has an immutable release tag, assets and source commit.

Stable and nightly have different product names and application identifiers,
so they can coexist. Only one application can own the serial port. Stable's
updater reads `releases/latest/download/latest.json`; nightly's reads
`releases/download/nightly/latest.json`. Switching channels means installing
the other app; automatic updates never change channels.

All three installers must build successfully before publication. A single
firmware build is shared across all platforms. The publisher checks commit,
firmware hash, version and channel consistency. Release assets include
`firmware.hex`, `firmware.json`, per-platform build metadata, and `SHA256SUMS.txt`.
The small nightly channel manifest is updated last and points to an immutable,
complete nightly release. An obsolete nightly is skipped if `main` advanced
before publication. Published stable versions cannot be overwritten.

An installer reflects one commit, not live repository contents. In-app updates
download a new signed build. The Teensy must then be flashed separately through
**App & firmware → Install bundled firmware**. The board reports its release
version and source commit in `HELLO`. Development builds report `local`.

## Automatic changelog

The public release list labels each version **New changes**, **Routine rebuild**,
or **Initial release**. Each release starts with a “What changed” section linked
to the previous published release on the same channel. Scheduled and manual
rebuilds of the same source commit explicitly say that there are no new source
commits; build tools or packaging can still differ. A scheduled build with new
commits is labeled **New changes**, just like a push-triggered build.

The publisher reads the previous release's `firmware.json` to find its source
commit, then lists changes from the source repository's first-parent history.
This avoids comparing unrelated tags in the separate downloads repository.
Drafts and the mutable `nightly` update pointer are excluded; stable and nightly
histories are compared separately. If history was rewritten or is unavailable,
notes say **Comparison unavailable**, rather than claiming nothing changed.
An API error or incompatible release metadata fails publication instead of
silently producing an incorrect changelog.

Write descriptive commit subjects for direct pushes and descriptive PR titles
for merges, for example `Fix firmware confirmation spacing`. These summaries
become public release notes; commit bodies, author emails, branch names and
private source links are not included. GitHub merge commits use the PR title
instead of the generic merge message. The notes include up to 100 summaries
and state the number of additional entries if that limit is exceeded.

The same summary is included in the updater manifest and the `nightly` channel
page, which links to the latest installers. No separate changelog file or
manual release-note editing is required.

## GitHub setup

Configure these in the **source** repository's Actions settings:

| Kind | Name | Value |
| --- | --- | --- |
| Variable | `RELEASE_REPOSITORY` | `venkatkidambi/teensy-panel-releases` |
| Secret | `RELEASE_TOKEN` | Fine-grained GitHub token scoped only to the public releases repository, with Contents: Read and write |
| Variable | `TAURI_SIGNING_PUBLIC_KEY` | Contents of the updater `.key.pub` file |
| Secret | `TAURI_SIGNING_PRIVATE_KEY` | Contents of the updater `.key` file |
| Secret, optional | `TAURI_SIGNING_PRIVATE_KEY_PASSWORD` | Password if the updater private key is encrypted |

The built-in `GITHUB_TOKEN` cannot write to another repository. `RELEASE_TOKEN`
is used by CI only and must never be embedded in the application. The publisher
uses the source repository's built-in token to check whether its commit is still
current. Renew an expiring release token before it expires.

Until `RELEASE_TOKEN` is added for a separate distribution repository, CI still
builds and uploads installers as private Actions artifacts, and explicitly skips
public publication. After adding the token, rerun the workflow or push the next
commit/tag to publish. Signing keys and the release destination alone do not
grant cross-repository write access.

Create the public releases repository with an initial README/default branch.
Its release tags refer to that distribution repository; `firmware.json` and the
release notes identify the exact **private source** commit. Only binaries,
release metadata and the upstream flasher's corresponding source are published.
The source repository, build token, signing key and application source are not
copied into the public repository.

Without `RELEASE_REPOSITORY`, releases go to the source repo. Private download
repositories automatically disable in-app updates because installed apps have no
GitHub credentials. Both signing key settings must be configured together, or
both omitted. Unsigned-update builds can still be installed manually, but they
cannot bootstrap themselves into signed updates: install a signed build once.

To generate an updater key locally using the existing npm dependencies:

```sh
mkdir -p .tools/release-signing
npx tauri signer generate --ci -w .tools/release-signing/updater.key
```

The `.tools` directory is gitignored. Back up this private key securely; losing
it prevents updates to installations that trust its public key. Do not regenerate
the key on each build. These updater signatures are independent of Apple and
Windows executable code signing.

## macOS signing and notarization

Without Apple credentials, CI builds an unnotarized app. Users will need to
approve it through macOS Privacy & Security; organization policies may block it.
This is suitable for initial internal testing, not a frictionless public launch.

To enable Developer ID signing and notarization, add these source-repo secrets:

- `APPLE_CERTIFICATE`: base64-encoded Developer ID Application `.p12` certificate.
- `APPLE_CERTIFICATE_PASSWORD`: password for that certificate.
- `APPLE_SIGNING_IDENTITY`: full Developer ID Application signing identity.
- `APPLE_ID`: Apple developer account email.
- `APPLE_PASSWORD`: Apple **app-specific password**, not the account password.
- `APPLE_TEAM_ID`: Apple developer team identifier.

Tauri imports the certificate, signs the application and its external flasher,
submits to Apple, and staples the notarization ticket. Set the optional
`RELEASE_PLATFORM_SIGNING` variable to describe the actual signing status in
release notes after validating it. Apple credentials are currently optional;
their absence does not disable updater signatures.

Windows installers currently have no Authenticode certificate configured, so
SmartScreen may prompt. The NSIS installer installs per user and embeds WebView2
for offline setup. Add your organization's Windows code-signing configuration
before broader distribution if needed.

## Creating a stable release

After changes are reviewed, merged and validated on the rig:

```sh
git tag v0.2.0
git push origin v0.2.0
```

Use the actual version from `desktop/tauri.conf.json`. This publishes all three
installers and marks the release as GitHub's latest stable version. Nightlies are
prereleases and never become latest stable. The daily schedule may be delayed by
GitHub and can be disabled by GitHub after inactivity; pushing main or manually
running the workflow also produces a nightly.

## Local package validation

`npm run build` still builds a normal developer executable without release
resources. CI prepares `desktop/tauri.release.json`, `desktop/resources/` and
`desktop/binaries/` (all gitignored), then runs:

```sh
npm run build -- --target aarch64-apple-darwin --config tauri.release.json --features bundled-release -- --locked
```

Use the matching native target on Windows or Intel Macs. `release.mjs metadata`
uses GitHub's ref, SHA and run number environment variables; `firmware` records
the HEX hash; `package TARGET` verifies firmware/source identity and stages the
native loader. `collect TARGET` checks the installer and update signatures;
`publish` uploads only after all platform jobs succeed.

Validate a release on a clean Windows machine and each supported Mac architecture:
install, launch without developer tools, verify the embedded version, connect to
a Teensy, flash the bundled firmware, reconnect, and verify a signed app update.
Exercise failure paths with no board attached and with an unresolved experiment.
No automated CI job flashes physical hardware. Save datasets first; flashing
resets volatile board state. The app blocks simultaneous updates and new serial
connections during installation, verifies the bundled firmware hash, and bounds
the flasher to three minutes. If flashing fails or times out, retry with the
Program button. Only connect the target Teensy when flashing.

## Bundled Teensy loader source

The loader is a separate executable built from
<https://github.com/PaulStoffregen/teensy_loader_cli> at
`03fca4156c244c7ad36bd368cf6e24531dbd566a`, with `desktop/loader-windows.patch`
adjusting two HID header paths for modern MinGW. The package includes
`teensy-loader-source.zip` (the actual patched source tree and build instructions) and `teensy-loader-GPL-3.0.txt`
inside its `release` resources. It is distributed under GPL-3.0 as stated
in its source. Build from the included archive with:

```sh
# macOS, using Xcode command-line tools
make OS=MACOSX CC=clang CFLAGS='-O2 -Wall -mmacosx-version-min=12.0'
# Windows, in an MSYS2 MINGW64 shell with make and mingw-w64-x86_64-gcc
make OS=WINDOWS CC=gcc CFLAGS='-O2 -Wall -static -static-libgcc'
```

See [Tauri's updater documentation](https://v2.tauri.app/plugin/updater/) and
[Mac signing documentation](https://v2.tauri.app/distribute/sign/macos/) for
credential management and platform details.
